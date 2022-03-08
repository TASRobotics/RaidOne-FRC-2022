package raidone.robot.auto.sequences;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.submodules.Intake;
import raidone.robot.submodules.Intake.IntakeState;

import java.util.Arrays;
import raidone.robot.auto.actions.SeriesAction;
import raidone.robot.auto.actions.LambdaAction;
import raidone.robot.auto.actions.DrivePath;

public class SmallBigBallSequence extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("SmallBigBall_0", AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL);
    private static final Trajectory path2 = PathPlanner.loadPath("SmallBigBall_1", AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL, true);

    private static final Intake intake = Intake.getInstance();

    public SmallBigBallSequence() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new LambdaAction(() -> intake.setState(IntakeState.DOWN)),
                new DrivePath(path1, true), 
                new LambdaAction(() -> intake.autoSet(-1)), 
                new LambdaAction(() -> Timer.delay(2)),
                new LambdaAction(() -> intake.autoSet(1)), 
                new DrivePath(path2, false), 
                new LambdaAction(() -> intake.autoSet(0)))
            )
        );
    }

    @Override
    public void onEnded() {
        System.out.println("SmallBigBallSequence ended!");
    }

    @Override
    public String getName() {
        return "Small Big Ball Sequence";
    }
}
