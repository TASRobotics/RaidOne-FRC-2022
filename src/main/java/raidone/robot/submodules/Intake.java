package raidone.robot.submodules;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidone.robot.Constants.IntakeConstants;
import raidone.robot.wrappers.InactiveDoubleSolenoid;
import raidone.robot.wrappers.LazyCANSparkMax;

public class Intake extends Submodule {
    public static class PeriodicIO {
        public double desiredIntakeSpeed = 0.0;
    }

    public static enum IntakeState {
        DOWN, UP;
    }

    /** Motors */
    private static final LazyCANSparkMax mLeftLeader = new LazyCANSparkMax(IntakeConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    private static final LazyCANSparkMax mRightFollower = new LazyCANSparkMax(IntakeConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    private PeriodicIO periodicIO = new PeriodicIO();

    /** Pneumatics */
    private static final InactiveDoubleSolenoid solenoid = new InactiveDoubleSolenoid(
        IntakeConstants.SOLENOID_DOWN_ID, IntakeConstants.SOLENOID_UP_ID);

    private Intake() {}
    private static Intake instance = null;
    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    @Override
    public void onInit() {
        mLeftLeader.restoreFactoryDefaults();
        mRightFollower.restoreFactoryDefaults();

        mRightFollower.follow(mLeftLeader);

        mLeftLeader.setInverted(false);
        mRightFollower.setInverted(true);
    }

    @Override
    public void onStart(double timestamp) {
        periodicIO = new PeriodicIO();

        stop();
    }

    @Override
    public void run() {
        mLeftLeader.set(periodicIO.desiredIntakeSpeed);
    }

    @Override
    public void stop() {
        periodicIO.desiredIntakeSpeed = 0.0;
        setState(IntakeState.UP);
    }

    /**
     * Sets the intake to the desired state.
     * 
     * @param speed desired intake speed
     */
    public void setPercentSpeed(double speed) {
        periodicIO.desiredIntakeSpeed = speed;
    }

    /**
     * Sets intake to the desired state.
     * 
     * @param state intake state
     */
    public void setState(IntakeState state) {
        switch(state) {
            case DOWN:
                solenoid.set(Value.kForward);
                break;
            case UP:
                solenoid.set(Value.kReverse);
                break;
        }
    }
}
