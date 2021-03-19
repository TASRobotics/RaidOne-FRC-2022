package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import raidone.robot.wrappers.LazyTalonFX;
import raidone.robot.wrappers.InactiveDoubleSolenoid;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import raidone.robot.Constants.ThroatConstants;
import raidone.robot.dashboard.Tab;

/**
 * Sucks BOBA balls and definitely not other balls
 */
public class Throat extends Submodule {

    private static enum Position {
        DOWN, UP
    }

    private static Throat instance = null;

    public static Throat getInstance() {
        if (instance == null) {
            instance = new Throat();
        }
        return instance;
    }

    private Throat() {
    }

    private LazyTalonFX left_throatMotor;
    private LazyTalonFX right_throatMotor; //leader

    private InactiveDoubleSolenoid solenoid;

    private double outputOpenLoop = 0.0;

    private Position position = Position.UP;

    private NetworkTableEntry throatPositionEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Throat Position", position.toString())
        .withPosition(6, 2)
        .withSize(1, 1)
        .getEntry();

    @Override
    public void onInit() {
        left_throatMotor = new LazyTalonFX(ThroatConstants.LEFT_MOTOR_ID);
        right_throatMotor = new LazyTalonFX(ThroatConstants.RIGHT_MOTOR_ID);
        left_throatMotor.configFactoryDefault();
        right_throatMotor.configFactoryDefault();
        left_throatMotor.setNeutralMode(ThroatConstants.NEUTRAL_MODE);
        right_throatMotor.setNeutralMode(ThroatConstants.NEUTRAL_MODE);

        left_throatMotor.setInverted(ThroatConstants.LEFT_INVERSION);
        right_throatMotor.setInverted(ThroatConstants.RIGHT_INVERSION);

        left_throatMotor.follow(right_throatMotor);

        solenoid = new InactiveDoubleSolenoid(ThroatConstants.THROAT_FORWARD_ID,
                ThroatConstants.THROAT_REVERSE_ID);
        setPosition(position);
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        right_throatMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        right_throatMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Loops up throat using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void loopBalls(double percentOutput) {
        outputOpenLoop = percentOutput;
    }

    public void setPosition(Position pos) {
        position = pos;
        throatPositionEntry.setString(position.toString());
        if (position == Position.DOWN) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.set(Value.kReverse);
        }
    }

    /**
     * Loops throat out or in depending on what state it is in.
     */
    public void invertStraw() {
        invertPos();
        setPosition(position);
    }

    private void invertPos() {
        if (position == Position.DOWN) {
            position = Position.UP;
            return;
        }
        position = Position.DOWN;
    }
}
