package raidone.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.auto.actions.DrivePath;
import raidone.robot.auto.actions.LambdaAction;
import raidone.robot.auto.actions.SeriesAction;
import raidone.robot.submodules.Intake;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class TestPath2Sequence extends AutoSequence{

    private static final Trajectory path = PathPlanner.loadPath("TestCurve", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private static final Intake intake = Intake.getInstance();

    public TestPath2Sequence() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new LambdaAction(() -> intake.autoSet(1)),
                new DrivePath(path, true)))
        );
    }

    @Override
    public void onEnded() {
        System.out.println("Test Path 2 Sequence ended!");
    }

    @Override
    public String getName() {
        return "Test Path 2 Sequence";
    }
    
}
