package raidone.robot.auto.actions;

import raidone.robot.submodules.Chassis;
import raidone.robot.utils.TimerBoolean;

public class SimpleMove implements Action {
    private final Chassis chassis = Chassis.getInstance();
    private TimerBoolean timer;
    private double leftPercentSpeed, rightPercentSpeed;

    public SimpleMove(double leftPercentSpeed, double rightPercentSpeed, double seconds) {
        this.leftPercentSpeed = leftPercentSpeed;
        this.rightPercentSpeed = rightPercentSpeed;
        timer = new TimerBoolean(seconds);
    }

    @Override
    public void start() {
        chassis.stop();
        chassis.setBrakeMode(true);
        timer.reset();
        timer.update(true);
    }

    @Override
    public void update() {
        chassis.setPercentSpeed(-leftPercentSpeed, -rightPercentSpeed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasDurationPassed();
    }

    @Override
    public void done() {
        chassis.setPercentSpeed(0, 0);
        chassis.stop();
    }
}
