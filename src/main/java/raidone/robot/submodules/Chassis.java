package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.Constants.ChassisConstants;
import raidone.robot.Constants;

import raidone.robot.wrappers.InactiveCompressor;
import raidone.robot.wrappers.InactiveDoubleSolenoid;

public class Chassis extends Submodule {
    /** Enum controlling gear shift */
    public static enum GearShift {
        HIGH_TORQUE, LOW_TORQUE, OFF
    }

    private final WPI_TalonSRX mLeftLeader = new WPI_TalonSRX(ChassisConstants.LEFT_LEADER_ID);
    private final WPI_VictorSPX mLeftFollowerA = new WPI_VictorSPX(ChassisConstants.LEFT_FOLLOWER_A_ID);
    private final WPI_VictorSPX mLeftFollowerB = new WPI_VictorSPX(ChassisConstants.LEFT_FOLLOWER_B_ID);

    private final WPI_TalonSRX mRightLeader = new WPI_TalonSRX(ChassisConstants.RIGHT_LEADER_ID);
    private final WPI_VictorSPX mRightFollowerA = new WPI_VictorSPX(ChassisConstants.RIGHT_FOLLOWER_A_ID);
    private final WPI_VictorSPX mRightFollowerB = new WPI_VictorSPX(ChassisConstants.RIGHT_FOLLOWER_B_ID);

    private final DifferentialDrive mChassis = new DifferentialDrive(mLeftLeader, mRightLeader);

    private final WPI_PigeonIMU mImu = new WPI_PigeonIMU(ChassisConstants.IMU_ID);

    private DifferentialDriveOdometry mOdometry;
    private RamseteController ramseteController;

    private final InactiveCompressor compressor = InactiveCompressor.getInstance();
    private final InactiveDoubleSolenoid shifter = new InactiveDoubleSolenoid(
        ChassisConstants.SHIFTER_HIGH_TORQUE_ID, 
        ChassisConstants.SHIFTER_LOW_TORQUE_ID);


    private Chassis() {}
    private static Chassis instance = null;
    public static Chassis getInstance() {
        if(instance == null) {
            instance = new Chassis();
        }
        return instance;
    }

    @Override
    public void onInit() {
        /** Config factory default for all motors */
        mLeftLeader.configFactoryDefault();
        mLeftFollowerA.configFactoryDefault();
        mLeftFollowerB.configFactoryDefault();
        mRightLeader.configFactoryDefault();
        mRightFollowerA.configFactoryDefault();
        mRightFollowerB.configFactoryDefault();

        /** Config followers */
        mLeftFollowerA.follow(mLeftLeader);
        mLeftFollowerB.follow(mLeftLeader);
        mRightFollowerA.follow(mRightLeader);
        mRightFollowerB.follow(mRightLeader);

        /** Inverts motors */
        mLeftLeader.setInverted(false);
        mLeftFollowerA.setInverted(InvertType.FollowMaster);
        mLeftFollowerB.setInverted(InvertType.FollowMaster);
        mRightLeader.setInverted(true);
        mRightFollowerA.setInverted(InvertType.FollowMaster);
        mRightFollowerB.setInverted(InvertType.FollowMaster);

        /** Inverts encoder */
        mLeftLeader.setSensorPhase(true);

        /** Sets feedback sensor */
        mLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
                                                 ChassisConstants.PID_LOOP_IDX, 
                                                 Constants.TIMEOUT_MS);
        mLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
                                                 ChassisConstants.PID_LOOP_IDX, 
                                                 Constants.TIMEOUT_MS);

        /** Config the peak and nominal outputs */
        mLeftLeader.configNominalOutputForward(0, Constants.TIMEOUT_MS);
        mLeftLeader.configNominalOutputReverse(0, Constants.TIMEOUT_MS);
        mLeftLeader.configPeakOutputForward(1, Constants.TIMEOUT_MS);
        mLeftLeader.configPeakOutputReverse(-1, Constants.TIMEOUT_MS);
        mRightLeader.configNominalOutputForward(0, Constants.TIMEOUT_MS);
        mRightLeader.configNominalOutputReverse(0, Constants.TIMEOUT_MS);
        mRightLeader.configPeakOutputForward(1, Constants.TIMEOUT_MS);
        mRightLeader.configPeakOutputReverse(-1, Constants.TIMEOUT_MS);

        /** Sets velocity PID gain */
        mLeftLeader.config_kF(ChassisConstants.PID_LOOP_IDX, ChassisConstants.LEFT_kF, Constants.TIMEOUT_MS);
        mLeftLeader.config_kP(ChassisConstants.PID_LOOP_IDX, ChassisConstants.LEFT_kP, Constants.TIMEOUT_MS);
        mLeftLeader.config_kD(ChassisConstants.PID_LOOP_IDX, ChassisConstants.LEFT_kD, Constants.TIMEOUT_MS);
        mRightLeader.config_kF(ChassisConstants.PID_LOOP_IDX, ChassisConstants.RIGHT_kF, Constants.TIMEOUT_MS);
        mRightLeader.config_kP(ChassisConstants.PID_LOOP_IDX, ChassisConstants.RIGHT_kP, Constants.TIMEOUT_MS);
        mRightLeader.config_kD(ChassisConstants.PID_LOOP_IDX, ChassisConstants.RIGHT_kD, Constants.TIMEOUT_MS);

        // Reset encoders
        resetEncoders();

        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));
        ramseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);

        changeShifterState(GearShift.LOW_TORQUE);
    }

    @Override
    public void onStart(double timestamp) {
        zero();
    }

    @Override
    public void run() {
        
    }

    /** Stops the compressor and all chassis motors */
    @Override
    public void stop() {
        mChassis.stopMotor();
        compressor.changeState();
    }

    /**
     * Sets desired velocity
     * 
     * @param left left vel
     * @param right right vel
     */
    public void setDesiredVelocity(double left, double right) {
        mLeftLeader.set(ControlMode.Velocity, left);
        mRightLeader.set(ControlMode.Velocity, right);
    }

    /**
     * Changes the shifter state
     * 
     * @param shift shifter setting
     */
    public void changeShifterState(GearShift shift) {
        if(shift == GearShift.LOW_TORQUE) {
            shifter.set(Value.kForward);
        } else if(shift == GearShift.HIGH_TORQUE) {
            shifter.set(Value.kReverse);
        } else {
            shifter.set(Value.kOff);
        }
    }

    /**
     * Calculates distance travelled on the left side based on encoder readings
     * 
     * @return distance travelled
     */
    public double getLeftEncoderDistance() {
        return mLeftLeader.getSelectedSensorPosition() * ChassisConstants.kEncoderDistancePerPulse;
    }

    /**
     * Calculates distance travelled on the left side based on encoder readings
     * 
     * @return distance travelled
     */
    public double getRightEncoderDistance() {
        return mRightLeader.getSelectedSensorPosition() * ChassisConstants.kEncoderDistancePerPulse;
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    /**
     * Returns left velocity
     * 
     * @return left velocity
     */
    private double getLeftVelocity() {
        return mLeftLeader.getSelectedSensorVelocity() * ChassisConstants.kEncoderDistancePerPulse * 10;
    }

    /**
     * Returns right velocity
     * 
     * @return right velocity
     */
    private double getRightVelocity() {
        return mRightLeader.getSelectedSensorVelocity() * ChassisConstants.kEncoderDistancePerPulse * 10;
    }

    /**
     * A better arcade drive
     * 
     * @param throttle usually the left y axis of a controller
     * @param turn usually the right x axis of a controllers
     * @param quickTurn basically an arcade drive switch
     */
    public void curvatureDrive(double throttle, double turn, boolean quickTurn) {
        mChassis.curvatureDrive(throttle, turn, quickTurn);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        mLeftLeader.setVoltage(leftVolts);
        mRightLeader.setVoltage(rightVolts);
        mChassis.feed();
    }

    /** Zeros all sensors */
    @Override
    public void zero() {
        resetEncoders();
        zeroHeading();
        resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    /** Resets drive encoders to 0 */
    public void resetEncoders() {
        mLeftLeader.setSelectedSensorPosition(0);
        mRightLeader.setSelectedSensorPosition(0);
    }

    /** Zeros IMU heading */
    public void zeroHeading() {
        mImu.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (mLeftLeader.getSelectedSensorPosition() + mRightLeader.getSelectedSensorPosition()) / 2.0;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        mChassis.setMaxOutput(maxOutput);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return rescale180(mImu.getRotation2d().getDegrees());
    }

    /**
     * Rescales an angle to [-180, 180]
     * 
     * @param angle the angle to be rescalled
     * @return rescalled angle
     */
    private double rescale180(double angle) {
        return angle - 360.0 * Math.floor((angle + 180.0) * (1.0 / 360.0));
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -mImu.getRate();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, new Rotation2d(getHeading()));
    }

    public void updateOdometry() {
        mOdometry.update(
            new Rotation2d(
                getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
    }
}
