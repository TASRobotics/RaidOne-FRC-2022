package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import raidone.robot.Constants.ChassisConstants;
import raidone.robot.pathing.TrajectoryFollower;
import raidone.robot.pathing.VelocityController;
import raidone.robot.Constants;

import raidone.robot.wrappers.InactiveCompressor;
import raidone.robot.wrappers.InactiveDoubleSolenoid;

public class Chassis extends Submodule {
    public static class PeriodicIO {
        // Inputs
        public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);

        public double leftPosition = 0; // in meters
        public double rightPosition = 0; // in meters

        public double leftVelocity = 0; // in m/s
        public double rightVelocity = 0; // in m/s

        // Outputs
        public double leftPercent = 0.0;
        public double rightPercent = 0.0;
    }

    /** Enum controlling gear shift */
    public static enum GearShift {
        HIGH_TORQUE, LOW_TORQUE, OFF
    }

    /** Enum controlling control state */
    public static enum ControlState {
        OPEN_LOOP, PATH_FOLLOWING
    }

    /** Motors */
    private final WPI_TalonSRX mLeftLeader = new WPI_TalonSRX(ChassisConstants.LEFT_LEADER_ID);
    private final WPI_VictorSPX mLeftFollowerA = new WPI_VictorSPX(ChassisConstants.LEFT_FOLLOWER_A_ID);
    private final WPI_VictorSPX mLeftFollowerB = new WPI_VictorSPX(ChassisConstants.LEFT_FOLLOWER_B_ID);

    private final WPI_TalonSRX mRightLeader = new WPI_TalonSRX(ChassisConstants.RIGHT_LEADER_ID);
    private final WPI_VictorSPX mRightFollowerA = new WPI_VictorSPX(ChassisConstants.RIGHT_FOLLOWER_A_ID);
    private final WPI_VictorSPX mRightFollowerB = new WPI_VictorSPX(ChassisConstants.RIGHT_FOLLOWER_B_ID);

    private final DifferentialDrive mChassis = new DifferentialDrive(mLeftLeader, mRightLeader);

    /** Sensors */
    private final WPI_PigeonIMU mImu = new WPI_PigeonIMU(ChassisConstants.IMU_ID);

    /** Controllers */
    private DifferentialDriveOdometry mOdometry;
    private TrajectoryFollower trajectoryFollower;
    private VelocityController leftVelController, rightVelController;
    private double leftPrevVel, rightPrevVel;

    private ControlState controlState = ControlState.OPEN_LOOP;
    private PeriodicIO periodicIO = new PeriodicIO();

    /** Pneumatics */
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
        // mLeftLeader.config_kF(ChassisConstants.PID_LOOP_IDX, ChassisConstants.LEFT_kF, Constants.TIMEOUT_MS);
        // mLeftLeader.config_kP(ChassisConstants.PID_LOOP_IDX, ChassisConstants.LEFT_kP, Constants.TIMEOUT_MS);
        // mLeftLeader.config_kD(ChassisConstants.PID_LOOP_IDX, ChassisConstants.LEFT_kD, Constants.TIMEOUT_MS);
        // mRightLeader.config_kF(ChassisConstants.PID_LOOP_IDX, ChassisConstants.RIGHT_kF, Constants.TIMEOUT_MS);
        // mRightLeader.config_kP(ChassisConstants.PID_LOOP_IDX, ChassisConstants.RIGHT_kP, Constants.TIMEOUT_MS);
        // mRightLeader.config_kD(ChassisConstants.PID_LOOP_IDX, ChassisConstants.RIGHT_kD, Constants.TIMEOUT_MS);

        mImu.configFactoryDefault();

        /** Config after imu init */
        trajectoryFollower = new TrajectoryFollower(ChassisConstants.DRIVE_KINEMATICS);
        leftVelController = new VelocityController(ChassisConstants.LEFT_kV, ChassisConstants.LEFT_kA, ChassisConstants.LEFT_kP);
        rightVelController = new VelocityController(ChassisConstants.RIGHT_kV, ChassisConstants.RIGHT_kA, ChassisConstants.RIGHT_kP);
        leftPrevVel = 0.0;
        rightPrevVel = 0.0;

        // Reset encoders
        // resetEncoders();
        zero();

        // mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));
        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeadingRad()));

        changeShifterState(GearShift.LOW_TORQUE);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        periodicIO = new PeriodicIO();

        stop();
        zero();
    }

    @Override
    public void run() {
        // switch(controlState) {
        //     case OPEN_LOOP: 
                mLeftLeader.set(ControlMode.PercentOutput, periodicIO.leftPercent);
                mRightLeader.set(ControlMode.PercentOutput, periodicIO.rightPercent);
            //     break;

            // case PATH_FOLLOWING: 
        // }
    }

    @Override
    public void update(double timestamp) {
        periodicIO.leftPosition = getLeftEncoderDistance();
        periodicIO.rightPosition = getRightEncoderDistance();

        periodicIO.leftVelocity = getLeftVelocity();
        periodicIO.rightVelocity = getRightVelocity();

        updateOdometry();

        if(controlState == ControlState.PATH_FOLLOWING) {
            double leftVel = trajectoryFollower.update(getPose()).leftMetersPerSecond;
            double rightVel = trajectoryFollower.update(getPose()).rightMetersPerSecond;

            /** Calculate accel */
            double leftAccel = leftVel - leftPrevVel;
            double rightAccel = rightVel - rightPrevVel;
            leftPrevVel = leftVel;
            rightPrevVel = rightVel;

            setPercentSpeed(
                leftVelController.update(leftVel, leftAccel, getLeftVelocity()), 
                rightVelController.update(rightVel, rightAccel, getRightVelocity()));
        }
    }

    /** Stops the compressor and all chassis motors */
    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;

        periodicIO.leftPercent = 0.0;
        periodicIO.rightPercent = 0.0;

        mLeftLeader.set(ControlMode.Disabled, 0.0);
        mRightLeader.set(ControlMode.Disabled, 0.0);
    }

    /**
     * Sets percent speed [-1, 1]
     * 
     * @param left left speed
     * @param right right speed
     */
    public void setPercentSpeed(double left, double right) {
        periodicIO.leftPercent = left;
        periodicIO.rightPercent = right;
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
     * Returns the heading of the robot in radians
     * 
     * @return the robot's heading in radians, from -pi to pi
     */
    public double getHeadingRad() {
        return Math.toRadians(getHeading());
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
        mOdometry.resetPosition(pose, new Rotation2d(getHeadingRad()));
    }

    /** Updates odom */
    private void updateOdometry() {
        mOdometry.update(
            new Rotation2d(
                getHeadingRad()), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    /**
     * Sets the brake mode to brake or coast.
     * 
     * @param brake whether to brake or not
     */
    public void setBrakeMode(boolean brake) {
        if (brake) {
            mRightLeader.setNeutralMode(NeutralMode.Brake);
            mLeftLeader.setNeutralMode(NeutralMode.Brake);
        } else {
            mRightLeader.setNeutralMode(NeutralMode.Coast);
            mLeftLeader.setNeutralMode(NeutralMode.Coast);
        }
    }

    /**
     * Makes the drive start following a Path.
     * 
     * @param path           the path to follow
     * @param zeroAllSensors whether to zero all sensors to the first point
     */
    public void setDrivePath(Trajectory trajectory) {
        if (trajectoryFollower != null) {
            // Stops the drive
            stop();

            // Reset & start trajectory follower
            trajectoryFollower.reset();
            trajectoryFollower.start(trajectory);

            controlState = ControlState.PATH_FOLLOWING;
        }
    }

    /**
     * Returns whether the drive has finished following a path.
     * 
     * @return if the drive is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (trajectoryFollower == null || controlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return trajectoryFollower.isFinished();
    }
}
