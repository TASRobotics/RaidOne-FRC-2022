package raidone.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.auto.actions.DrivePath;
import raidone.robot.auto.actions.LambdaAction;
import raidone.robot.auto.actions.SeriesAction;
import raidone.robot.submodules.Intake;
import raidone.robot.submodules.Intake.IntakeState;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class BigBallSequence extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("BigBall_0", AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL);
    private static final Trajectory path2 = PathPlanner.loadPath("BigBall_1", AutoConstants.MAX_VEL, AutoConstants.MAX_ACCEL, true);


    private static final Intake intake = Intake.getInstance();

    public BigBallSequence() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new LambdaAction(() -> intake.setState(IntakeState.DOWN)),
                new DrivePath(path1, true), 
                new LambdaAction(() -> intake.autoSet(-1)), 
                new LambdaAction(() -> Timer.delay(3)),
                new LambdaAction(() -> intake.autoSet(1)),
                new DrivePath(path2, false))
            )
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
