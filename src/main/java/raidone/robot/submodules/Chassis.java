package raidone.robot.submodules;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.Constants.ChassisConstants;
import raidone.robot.pathing.TrajectoryFollower;
import raidone.robot.pathing.VelocityController;
import raidone.robot.utils.JoystickUtils;
import raidone.robot.Constants;

import raidone.robot.wrappers.InactiveCompressor;
import raidone.robot.wrappers.InactiveDoubleSolenoid;
import raidone.robot.wrappers.LogicalTalonSRX;

public class Chassis extends Submodule {
    public static class PeriodicIO {
        // Inputs
        public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);

        public double leftPosition = 0; // in meters
        public double rightPosition = 0; // in meters
        public Rotation2d heading = new Rotation2d(0);

        public double actualLeftVelocity = 0; // in m/s
        public double actualRightVelocity = 0; // in m/s

        public double x = 0;
        public double y = 0;
        public Rotation2d rotation = new Rotation2d(0);

        // Outputs
        public double leftPercent = 0.0;
        public double rightPercent = 0.0;

        public double desiredLeftVelocity = 0.0;
        public double desiredRightVelocity = 0.0;

        public double leftFF = 0.0;
        public double rightFF = 0.0;
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
    private final LogicalTalonSRX mLeftLeader = new LogicalTalonSRX(ChassisConstants.LEFT_LEADER_ID);
    private final WPI_VictorSPX mLeftFollowerA = new WPI_VictorSPX(ChassisConstants.LEFT_FOLLOWER_A_ID);
    private final WPI_VictorSPX mLeftFollowerB = new WPI_VictorSPX(ChassisConstants.LEFT_FOLLOWER_B_ID);

    private final LogicalTalonSRX mRightLeader = new LogicalTalonSRX(ChassisConstants.RIGHT_LEADER_ID);
    private final WPI_VictorSPX mRightFollowerA = new WPI_VictorSPX(ChassisConstants.RIGHT_FOLLOWER_A_ID);
    private final WPI_VictorSPX mRightFollowerB = new WPI_VictorSPX(ChassisConstants.RIGHT_FOLLOWER_B_ID);


    /** Sensors */
    private final WPI_Pigeon2 mImu = new WPI_Pigeon2(ChassisConstants.IMU_ID);

    /** Controllers */
    private DifferentialDriveOdometry mOdometry;
    private TrajectoryFollower trajectoryFollower;
    // private VelocityController leftVelController, rightVelController;
    private VelocityController velocityController;
    private double leftPrevVel, rightPrevVel;

    private ControlState controlState = ControlState.OPEN_LOOP;
    private PeriodicIO periodicIO = new PeriodicIO();

    /** Pneumatics */
    private InactiveCompressor compressor;
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
        compressor = InactiveCompressor.getInstance();
        /** Config factory default for all motors */
        mLeftLeader.configFactoryDefault();
        mLeftFollowerA.configFactoryDefault();
        mLeftFollowerB.configFactoryDefault();
        mRightLeader.configFactoryDefault();
        mRightFollowerA.configFactoryDefault();
        mRightFollowerB.configFactoryDefault();

        /** Config factory default for sensors */
        mImu.configFactoryDefault();

        /** Config followers */
        mLeftFollowerA.follow(mLeftLeader);
        mLeftFollowerB.follow(mLeftLeader);
        mRightFollowerA.follow(mRightLeader);
        mRightFollowerB.follow(mRightLeader);

        /** Inverts motors */
        mLeftLeader.setInverted(true);
        mLeftFollowerA.setInverted(InvertType.FollowMaster);
        mLeftFollowerB.setInverted(InvertType.FollowMaster);
        mRightLeader.setInverted(false);
        mRightFollowerA.setInverted(InvertType.FollowMaster);
        mRightFollowerB.setInverted(InvertType.FollowMaster);

        /** Inverts encoder */
        mRightLeader.setSensorPhase(false);
        mLeftLeader.setSensorPhase(false);

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

        /** Config voltage compensation */
        mLeftLeader.configVoltageCompSaturation(Constants.VOLTAGE_COMPENSATION, Constants.TIMEOUT_MS);
        mLeftLeader.enableVoltageCompensation(true);
        mRightLeader.configVoltageCompSaturation(Constants.VOLTAGE_COMPENSATION, Constants.TIMEOUT_MS);
        mRightLeader.enableVoltageCompensation(true);

        /** Config ramp rate */
        mLeftLeader.configOpenloopRamp(ChassisConstants.RAMP_RATE, Constants.TIMEOUT_MS);
        mLeftLeader.configClosedloopRamp(0);
        mRightLeader.configOpenloopRamp(ChassisConstants.RAMP_RATE, Constants.TIMEOUT_MS);
        mRightLeader.configClosedloopRamp(0);

        mLeftLeader.configContinuousCurrentLimit(30, Constants.TIMEOUT_MS);
        mLeftLeader.configPeakCurrentLimit(80, Constants.TIMEOUT_MS);
        mLeftLeader.configPeakCurrentDuration(1000, Constants.TIMEOUT_MS);
        mRightLeader.configContinuousCurrentLimit(30, Constants.TIMEOUT_MS);
        mRightLeader.configPeakCurrentLimit(80, Constants.TIMEOUT_MS);
        mRightLeader.configPeakCurrentDuration(1000, Constants.TIMEOUT_MS);

        /** Config Talon PID */
        // ! change !
        mLeftLeader.config_kP(ChassisConstants.PID_LOOP_IDX, 
                              ChassisConstants.kP, 
                              Constants.TIMEOUT_MS);
        mRightLeader.config_kP(ChassisConstants.PID_LOOP_IDX, 
                               ChassisConstants.kP, 
                               Constants.TIMEOUT_MS);

        /** Config after imu init */
        trajectoryFollower = new TrajectoryFollower(ChassisConstants.DRIVE_KINEMATICS);
        // leftVelController = new VelocityController(ChassisConstants.LEFT_kV, ChassisConstants.LEFT_kA, ChassisConstants.LEFT_kP);
        // rightVelController = new VelocityController(ChassisConstants.RIGHT_kV, ChassisConstants.RIGHT_kA, ChassisConstants.RIGHT_kP);
        velocityController = new VelocityController(ChassisConstants.kS, ChassisConstants.kV, ChassisConstants.kA, ChassisConstants.kP);
        leftPrevVel = 0.0;
        rightPrevVel = 0.0;

        // Reset sensors (must happen before odom init)
        zero();

        mOdometry = new DifferentialDriveOdometry(periodicIO.heading);

        setBrakeMode(true);
        changeShifterState(GearShift.LOW_TORQUE);

        /** Camera */
        // UsbCamera cam1 =  CameraServer.startAutomaticCapture(0);
        // cam1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);


        // Logger.configureLoggingAndConfig(this, false);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        periodicIO = new PeriodicIO();

        stop();
        zero();
        resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

        changeShifterState(GearShift.LOW_TORQUE);
        setBrakeMode(true);
    }

    @Override
    public void run() {
        switch(controlState) {
            case OPEN_LOOP:
                mLeftLeader.set(ControlMode.PercentOutput, periodicIO.leftPercent);
                mRightLeader.set(ControlMode.PercentOutput, periodicIO.rightPercent);
                break;

            case PATH_FOLLOWING:
                mLeftLeader.set(ControlMode.Velocity, periodicIO.desiredLeftVelocity, DemandType.ArbitraryFeedForward, periodicIO.leftFF);
                mRightLeader.set(ControlMode.Velocity, periodicIO.desiredRightVelocity, DemandType.ArbitraryFeedForward, periodicIO.rightFF);
                break;
        }
    }

    @Override
    public void update(double timestamp) {
        periodicIO.leftPosition = mLeftLeader.getSelectedSensorPosition() * ChassisConstants.kEncoderDistancePerPulse;
        periodicIO.rightPosition = mRightLeader.getSelectedSensorPosition() * ChassisConstants.kEncoderDistancePerPulse;

        // ! change
        // periodicIO.actualLeftVelocity = mLeftLeader.getSelectedSensorVelocity() * ChassisConstants.kEncoderDistancePerPulse * 10;
        // periodicIO.actualRightVelocity = mRightLeader.getSelectedSensorVelocity() * ChassisConstants.kEncoderDistancePerPulse * 10;
        periodicIO.actualLeftVelocity = mLeftLeader.getSelectedSensorVelocity() * ChassisConstants.kEncoderDistancePerPulse;
        periodicIO.actualRightVelocity = mRightLeader.getSelectedSensorVelocity() * ChassisConstants.kEncoderDistancePerPulse;

        periodicIO.heading = Rotation2d.fromDegrees(rescale180(mImu.getRotation2d().getDegrees()));

        Pose2d updatedPose = updateOdometry();
        periodicIO.x = updatedPose.getX();
        periodicIO.y = updatedPose.getY();
        periodicIO.rotation = updatedPose.getRotation();
        SmartDashboard.putNumber("actual left vel", periodicIO.actualLeftVelocity);
        SmartDashboard.putNumber("actual right vel", periodicIO.actualRightVelocity);
        SmartDashboard.putNumber("heading", periodicIO.heading.getDegrees());

        SmartDashboard.putNumber("left enc", mLeftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right enc", mRightLeader.getSelectedSensorPosition());

        SmartDashboard.putNumber("left vel 1234", mLeftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right vel 1234", mRightLeader.getSelectedSensorVelocity());


        if(controlState == ControlState.PATH_FOLLOWING) {
            /** WHY DO I NEED TO MAKE THIS NEGATIVE!?! */
            double leftVel = -trajectoryFollower.update(updatedPose).leftMetersPerSecond;
            double rightVel = -trajectoryFollower.update(updatedPose).rightMetersPerSecond;
            // double rightVel = trajectoryFollower.update(updatedPose).leftMetersPerSecond;
            // double leftVel = trajectoryFollower.update(updatedPose).rightMetersPerSecond;

            /** Calculate accel */
            double leftAccel = leftVel - leftPrevVel;
            double rightAccel = rightVel - rightPrevVel;
            leftPrevVel = leftVel;
            rightPrevVel = rightVel;

            SmartDashboard.putNumber("desired left vel", -leftVel);
            SmartDashboard.putNumber("desired right vel", -rightVel);

            // periodicIO.leftFF = leftVelController.updateFF(leftVel, leftAccel);
            // periodicIO.rightFF = rightVelController.updateFF(rightVel, rightAccel);
            periodicIO.leftFF = velocityController.updateFF(leftVel, leftAccel);
            periodicIO.rightFF = velocityController.updateFF(rightVel, rightAccel);

            setVelocity(leftVel, rightVel);
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

        // compressor.changeState();
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
     * Sets velocity speed [-1, 1]
     * 
     * @param left left speed
     * @param right right speed
     */
    public void setVelocity(double left, double right) {
        periodicIO.desiredLeftVelocity = left;
        periodicIO.desiredRightVelocity = right;
    }

    /**
     * Changes the shifter state
     * 
     * @param shift shifter setting
     */
    public void changeShifterState(GearShift shift) {
        if(shift == GearShift.HIGH_TORQUE) {
            shifter.set(Value.kReverse);
        } else if(shift == GearShift.LOW_TORQUE) {
            shifter.set(Value.kForward);
        } else {
            shifter.set(Value.kOff);
        }
    }

    /**
     * A better arcade drive
     * 
     * @param throttle usually the left y axis of a controller
     * @param turn usually the right x axis of a controllers
     * @param quickTurn basically an arcade drive switch
     */
    public void curvatureDrive(double throttle, double turn, boolean quickTurn) {
        /** Set deadband to all inputs */
        // throttle = JoystickUtils.deadband(JoystickUtils.monomialScale(throttle, ChassisConstants.MONOMIAL_SCALE, 1));
        // turn = JoystickUtils.deadband(JoystickUtils.monomialScale(turn, ChassisConstants.MONOMIAL_SCALE, 1));

        throttle = JoystickUtils.deadband(throttle);
        turn = JoystickUtils.deadband(turn);

        // Compute velocity, right stick = curvature if no quickturn, else power
        double leftSpeed = throttle + (quickTurn ? turn : Math.abs(throttle) * turn);
        double rightSpeed = throttle - (quickTurn ? turn : Math.abs(throttle) * turn);
    
        // Normalize velocity
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }
        periodicIO.leftPercent = leftSpeed;
        periodicIO.rightPercent = rightSpeed;
    }

    /**
     * Aracde drive
     * 
     * @param throttle forward
     * @param turn turn
     */
    public void arcadeDrive(double throttle, double turn) {
        throttle = JoystickUtils.deadband(JoystickUtils.monomialScale(throttle, ChassisConstants.MONOMIAL_SCALE, 1));
        turn = JoystickUtils.deadband(JoystickUtils.monomialScale(turn, ChassisConstants.MONOMIAL_SCALE, 1));
        periodicIO.leftPercent = throttle + turn;
        periodicIO.rightPercent = throttle - turn;
    }

    /**
     * Tank drive
     * 
     * @param left percent speed
     * @param right percent speed
     */
    public void tankDrive(double left, double right) {
        // left = JoystickUtils.deadband(JoystickUtils.monomialScale(left, ChassisConstants.MONOMIAL_SCALE, 1));
        // right = JoystickUtils.deadband(JoystickUtils.monomialScale(right, ChassisConstants.MONOMIAL_SCALE, 1));
        left = JoystickUtils.deadband(left);
        right = JoystickUtils.deadband(right);
        periodicIO.leftPercent = left;
        periodicIO.rightPercent = right;
    }

    /** Zeros all sensors */
    @Override
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
     * Returns Periodic IO
     * 
     * @return current periodic io
     */
    public PeriodicIO getPeriodicIO() {
        return periodicIO;
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
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, periodicIO.heading);
    }

    /** Updates odom */
    private Pose2d updateOdometry() {
        return mOdometry.update(periodicIO.heading, periodicIO.leftPosition, periodicIO.rightPosition);
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
