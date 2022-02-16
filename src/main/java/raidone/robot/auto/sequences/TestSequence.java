package raidone.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.auto.actions.DrivePath;
import raidone.robot.auto.actions.SeriesAction;

public class TestSequence extends AutoSequence {
    private static final Trajectory path = PathPlanner.loadPath("TestCurve", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    public TestSequence() {}

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new DrivePath(path, true)
        )));
    }

    @Override
    public void onEnded() {
        System.out.println("TestSequence ended!");
    }

    @Override
    public String getName() {
        return "Test Sequence";
    }
}
