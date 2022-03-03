package raidone.robot.auto.sequences;

import java.util.Arrays;

import raidone.robot.auto.actions.LambdaAction;
import raidone.robot.auto.actions.SeriesAction;
import raidone.robot.auto.actions.SimpleMove;
import raidone.robot.submodules.Intake;

public class AutoLineSequence extends AutoSequence {
    private final Intake intake = Intake.getInstance();

    public AutoLineSequence() {}

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new LambdaAction(() -> intake.setPercentSpeed(-1)), 
            new SimpleMove(0.5, 0.5, 2)
        )));
    }

    @Override
    public void onEnded() {
        System.out.println("Auto Line Sequence ended!");
    }

    @Override
    public String getName() {
        return "Auto Line Sequence";
    }
}
