package raidone.robot.auto.actions;

/** Shamelessly stolen from Raid Zero 2020 code */
public class LambdaAction implements Action {

    public interface VoidInterface {
        void f();
    }

    VoidInterface mF;

    public LambdaAction(VoidInterface f) {
        this.mF = f;
    }

    @Override
    public void start() {
        mF.f();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}