package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.robot.Constants.ChassisConstants;
import raidone.robot.wrappers.InactiveCompressor;
import raidone.robot.wrappers.InactiveDoubleSolenoid;

public class Chassis extends SubsystemBase {
    /** Enum controlling gear shift */
    public static enum GearShift {
        HIGH_TORQUE, LOW_TORQUE, OFF
    }

    private final WPI_TalonSRX mLeftLeader = new WPI_TalonSRX(ChassisConstants.LEFT_LEADER_ID);
    private final WPI_TalonSRX mLeftFollowerA = new WPI_TalonSRX(ChassisConstants.LEFT_FOLLOWER_A_ID);
    private final WPI_TalonSRX mLeftFollowerB = new WPI_TalonSRX(ChassisConstants.LEFT_FOLLOWER_B_ID);

    private final WPI_TalonSRX mRightLeader = new WPI_TalonSRX(ChassisConstants.RIGHT_LEADER_ID);
    private final WPI_TalonSRX mRightFollowerA = new WPI_TalonSRX(ChassisConstants.RIGHT_FOLLOWER_A_ID);
    private final WPI_TalonSRX mRightFollowerB = new WPI_TalonSRX(ChassisConstants.RIGHT_FOLLOWER_B_ID);

    private final MotorControllerGroup mLeftMotors = 
        new MotorControllerGroup(
            mLeftLeader, 
            mLeftFollowerA, 
            mLeftFollowerB);
    private final MotorControllerGroup mRightMotors = 
        new MotorControllerGroup(
            mRightLeader, 
            mRightFollowerA, 
            mRightFollowerB);

    private final DifferentialDrive mChassis = new DifferentialDrive(mLeftMotors, mRightMotors);

    private final WPI_PigeonIMU mImu = new WPI_PigeonIMU(ChassisConstants.IMU_ID);

    private DifferentialDriveOdometry mOdometry;

    private final InactiveCompressor compressor = InactiveCompressor.getInstance();
    private final InactiveDoubleSolenoid shifter = new InactiveDoubleSolenoid(
        ChassisConstants.SHIFTER_HIGH_TORQUE_ID, 
        ChassisConstants.SHIFTER_LOW_TORQUE_ID);

    private static Chassis instance = null;
    public static Chassis getInstance() {
        if(instance == null) {
            instance = new Chassis();
        }
        return instance;
    }

    
    public void onInit() {
        // Inverts motors and encoders
        mRightMotors.setInverted(true);
        mLeftLeader.setSensorPhase(true);

        // Reset encoders
        resetEncoders();

        // Init odom
        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));

        changeShifterState(GearShift.LOW_TORQUE);
    }

    
    public void run() {
        // Runs odometry in the background
        mOdometry.update(
            new Rotation2d(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    /** Stops the compressor and all chassis motors */
    
    public void stop() {
        mChassis.stopMotor();
        compressor.changeState();
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

    public DifferentialDrive getDifferentialDrive() {
        return mChassis;
    }

    /**
     * Calculates distance travelled on the left side based on encoder readings
     * 
     * @return distance travelled
     */
    private double getLeftEncoderDistance() {
        return mLeftLeader.getSelectedSensorPosition() * ChassisConstants.kEncoderDistancePerPulse;
    }

    /**
     * Calculates distance travelled on the left side based on encoder readings
     * 
     * @return distance travelled
     */
    private double getRightEncoderDistance() {
        return mRightLeader.getSelectedSensorPosition() * ChassisConstants.kEncoderDistancePerPulse;
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
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, mImu.getRotation2d());
    }

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
        mLeftMotors.setVoltage(leftVolts);
        mRightMotors.setVoltage(rightVolts);
        mChassis.feed();
    }

    /** Zeros all sensors */
    
    public void zero() {
        resetEncoders();
        zeroHeading();
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
}
