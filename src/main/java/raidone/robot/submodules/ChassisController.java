package raidone.robot.submodules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

import raidone.robot.Constants.ChassisConstants;
import raidone.robot.Constants.AutoConstants;

public class ChassisController extends Submodule {
    private Chassis chassis = Chassis.getInstance();

    private DifferentialDriveOdometry mOdometry;
    
    private DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private TrajectoryConfig config;
    private RamseteController ramseteController;
    private PIDController PID;
    private Trajectory path;

    private static ChassisController instance = null;
    public static ChassisController getInstance() {
        if(instance == null) {
            instance = new ChassisController();
        }
        return instance;
    }

    @Override 
    public void onInit() {
        // Init odom
        mOdometry = new DifferentialDriveOdometry(new Rotation2d(chassis.getHeading()));
        // Create a voltage constraint to ensure we don't accelerate too fast
        autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    ChassisConstants.ksVolts,
                    ChassisConstants.kvVoltSecondsPerMeter,
                    ChassisConstants.kaVoltSecondsSquaredPerMeter),
                ChassisConstants.kDriveKinematics,
                10);
        
        // Create config for trajectory
        config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(ChassisConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        ramseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
        // PID = new PIDController(ChassisConstants.kPDriveVel, 0.0, 0.0);
    }

    @Override
    public void onStart(double timestamp) {
        zero();
    }

    @Override
    public void run() {
        // Runs odometry in the background
        mOdometry.update(
            new Rotation2d(
                chassis.getHeading()), chassis.getLeftEncoderDistance(), chassis.getRightEncoderDistance());
    }

    @Override
    public void update(double timestamp) {
        // ramsete
    }

    @Override
    public void stop() {
        chassis.stop();
    }

    @Override
    public void zero() {
        chassis.zero();
        resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    public void setPath(Trajectory path) {
        this.path = path;
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
        chassis.resetEncoders();
        mOdometry.resetPosition(pose, new Rotation2d(chassis.getHeading()));
    }
}
