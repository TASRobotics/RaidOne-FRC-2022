package raidone.robot.pathing;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

/**
 * References:
 * - https://github.com/STMARobotics/frc-7028-2020/blob/master/src/main/java/frc/robot/subsystems/DriveTrainSubsystem.java
 * - https://www.chiefdelphi.com/t/path-trajectory-with-talonsrx-help/372940
 * - Louis (for making this this)
 */
public class TrajectoryFollower {

    private RamseteController controller;
    private DifferentialDriveKinematics kinematics;
    private Trajectory currentTrajectory;

    private Timer timer = new Timer();

    /**
     * Constructs a trajectory follower with default ramsete constants.
     * 
     * @param kinematics kinematics for the drive
     */
    public TrajectoryFollower(DifferentialDriveKinematics kinematics) {
        this(kinematics, 2.0, 0.7);
    }

    /**
     * Constructs a trajectory follower with custom ramsete constants.
     * 
     * @param kinematics kinematics for the drive
     * @param b          Tuning parameter (b > 0) for which larger values 
     *                   make convergence more aggressive like a kP term.
     * @param zeta       Tuning parameter (0 < zeta < 1) for which larger 
     *                   values provide more damping in response.
     */
    public TrajectoryFollower(DifferentialDriveKinematics kinematics, double b, double zeta) {
        controller = new RamseteController(b, zeta);
        controller.setEnabled(true);
        this.kinematics = kinematics;
    }

    /**
     * Updates the Ramsete controller and returns the adjusted velocities.
     * 
     * @param currentPose the current pose of the drive (from odometry)
     * @return adjusted wheel velocities in m/s
     */
    public DifferentialDriveWheelSpeeds update(Pose2d currentPose) {
        double time = timer.get();
        PathPlannerState sampled = (PathPlannerState) currentTrajectory.sample(time);
        System.out.println("Sampled: " + sampled.toString() + " | Actual: " + currentPose.toString());
        // var targetWheelSpeeds = kinematics.toWheelSpeeds(
        //     controller.calculate(currentPose, sampled)
        // );
        var targetWheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(sampled.velocityMetersPerSecond, 0, sampled.angularVelocity.getRadians()));
        return targetWheelSpeeds;
    }

    /**
     * Resets the trajectory follower.
     */
    public void reset() {
        currentTrajectory = null;
        timer.reset();
    }

    /**
     * Starts the timer for trajectory following.
     * 
     * @param trajectory the trajectory to follow
     */
    public void start(Trajectory trajectory) {
        currentTrajectory = trajectory;
        timer.start();
    }

    /**
     * Returns whether the trajectory has finished or not.
     * 
     * @return if the drive is finished or not
     */
    public boolean isFinished() {
        // TODO: Compare poses instead of using time
        return timer.advanceIfElapsed(currentTrajectory.getTotalTimeSeconds());
    }
}