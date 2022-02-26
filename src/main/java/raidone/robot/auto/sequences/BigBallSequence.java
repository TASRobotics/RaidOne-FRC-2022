package raidone.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.auto.actions.DrivePath;
import raidone.robot.auto.actions.SeriesAction;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class BigBallSequence extends AutoSequence {
    private static final Trajectory path = PathPlanner.loadPath("TestCurve", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    public BigBallSequence() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new DrivePath(path, true)))
        );
    }

    @Override
    public void onEnded() {
        System.out.println("BigBallSequence ended!");
    }

    @Override
    public String getName() {
        return "Big Ball Sequence";
    }
}
