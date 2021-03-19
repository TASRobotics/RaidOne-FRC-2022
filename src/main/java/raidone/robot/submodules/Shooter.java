package raidone.robot.submodules;

import raidone.robot.wrappers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import raidone.robot.Constants;
import raidone.robot.Constants.ShooterConstants;
import raidone.robot.dashboard.Tab;

public class Shooter extends Submodule {

    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
    }

    private LazyTalonFX left_shooterMotor;
    private LazyTalonFX right_shooterMotor; //leader

    private double outputPercentSpeed = 0.0; 

    private NetworkTableEntry shooterVelocityEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Shooter Vel", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(0, 2)
        .getEntry();
    private NetworkTableEntry shooterUpToSpeedEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Up To Speed", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(1, 2)
        .getEntry();

    @Override
    public void onInit() {
        left_shooterMotor = new LazyTalonFX(ShooterConstants.LEFT_MOTOR_ID);
        right_shooterMotor = new LazyTalonFX(ShooterConstants.RIGHT_MOTOR_ID);
        left_shooterMotor.configFactoryDefault();
        right_shooterMotor.configFactoryDefault();
        left_shooterMotor.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
        left_shooterMotor.setInverted(ShooterConstants.LEFT_INVERSION);
        right_shooterMotor.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
        right_shooterMotor.setInverted(ShooterConstants.RIGHT_INVERSION);

        left_shooterMotor.follow(right_shooterMotor);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = ShooterConstants.K_F;
        config.slot0.kP = ShooterConstants.K_P;
        config.slot0.kI = ShooterConstants.K_I;
        config.slot0.kD = ShooterConstants.K_D;
        config.slot0.integralZone = ShooterConstants.K_INTEGRAL_ZONE;

        right_shooterMotor.configAllSettings(config);
    }

    @Override
    public void onStart(double timestamp) {
        outputPercentSpeed = 0.0;
        zero();
    }

    @Override
    public void update(double timestamp) {
        shooterVelocityEntry.setNumber(right_shooterMotor.getSelectedSensorVelocity());
        shooterUpToSpeedEntry.setBoolean(isUpToSpeed());
    }

    @Override
    public void run() {
        if (Math.abs(outputPercentSpeed) < 0.1) {
            stop();
        } else {
            right_shooterMotor.set(ControlMode.Velocity,
                    outputPercentSpeed * ShooterConstants.FAKE_MAX_SPEED);
        }
    }

    @Override
    public void stop() {
        outputPercentSpeed = 0.0;
        right_shooterMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        right_shooterMotor.getSensorCollection().setIntegratedSensorPosition(0.0, Constants.TIMEOUT_MS);
    }

    /**
     * Fires up the shooter.
     * 
     * @param percentSpeed speed of the shooter in [-1.0, 1.0]
     * @param freeze       whether to disregard the speed and keep the previous
     *                     speed
     */
    public void shoot(double percentSpeed, boolean freeze) {
        if (freeze) {
            return;
        }
        outputPercentSpeed = percentSpeed;
    }

    /**
     * Returns whether the shooter is up to the setpoint speed.
     * 
     * @return whether the shooter is up to speed
     */
    public boolean isUpToSpeed() {
        return Math.abs(outputPercentSpeed) > 0.1 &&
               Math.abs(right_shooterMotor.getClosedLoopError()) < ShooterConstants.ERROR_TOLERANCE;
    }
}
