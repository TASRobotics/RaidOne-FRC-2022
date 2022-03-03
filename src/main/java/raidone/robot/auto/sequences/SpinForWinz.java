package raidone.robot.auto.sequences;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.submodules.Intake;

import java.util.Arrays;
import raidone.robot.auto.actions.SeriesAction;
import raidone.robot.auto.actions.SimpleMove;
import raidone.robot.auto.actions.LambdaAction;
import raidone.robot.auto.actions.DrivePath;

public class SpinForWinz extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("SpinForWinz_0", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);


    private static final Intake intake = Intake.getInstance();

    public SpinForWinz() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new DrivePath(path1, true), 
                new SimpleMove(0.5, -0.5, 10))
            )
        );
    }

    @Override
    public void onEnded() {
        System.out.println("SpinForWinz ended!");
    }

    @Override
    public String getName() {
        return "Spin for Winz Sequence";
    }
}
