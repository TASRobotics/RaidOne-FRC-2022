package raidone.robot.auto.sequences;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.submodules.Intake;

import java.util.Arrays;
import raidone.robot.auto.actions.SeriesAction;
import raidone.robot.auto.actions.LambdaAction;
import raidone.robot.auto.actions.DrivePath;

public class SmallBallSequence extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("SmallBall_0", AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL);


    private static final Intake intake = Intake.getInstance();

    public SmallBallSequence() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new LambdaAction(() -> intake.autoSet(1)),
                new DrivePath(path1, true), 
                new LambdaAction(() -> intake.autoSet(0)))
            )
        );
    }

    @Override
    public void onEnded() {
        System.out.println("SmallBallSequence ended!");
    }

    @Override
    public String getName() {
        return "Small Ball Sequence";
    }
}