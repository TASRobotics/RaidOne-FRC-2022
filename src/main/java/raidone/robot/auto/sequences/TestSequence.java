package raidone.robot.auto.sequences;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.submodules.ChassisController;

public class TestSequence extends AutoSequence {
    private static ChassisController chassisController = ChassisController.getInstance();
    private Trajectory path;

    public TestSequence() {}

    @Override
    public void sequence() {
        path = PathPlanner.loadPath("TestCurve", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        chassisController.setTolerance(new Pose2d(0.5, 0.5, new Rotation2d(10)));
        chassisController.setPath(path);
    }

    // @Override
    // public void onEnded() {
        
    // }

    @Override
    public String getName() {
        return "Test Sequence";
    }
}
