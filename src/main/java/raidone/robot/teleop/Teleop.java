package raidone.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.Constants;
import raidone.robot.submodules.Chassis;
import raidone.robot.submodules.Intake;
import raidone.robot.submodules.Chassis.GearShift;

public class Teleop {

    private static Teleop instance = null;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private Teleop() {
    }

    private XboxController master = new XboxController(0);

    private static Chassis chassis = Chassis.getInstance();
    private static Intake intake = Intake.getInstance();

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        // chassis.changeShifterState(GearShift.LOW_TORQUE);

        chassis.zero();
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        chassis.curvatureDrive(-master.getRawAxis(1), master.getRawAxis(2), Math.abs(master.getRawAxis(1)) < Constants.DEADBAND);
        // if(master.getLeftBumper()) chassis.changeShifterState(GearShift.HIGH_TORQUE);
        // else if(master.getRightBumper()) chassis.changeShifterState(GearShift.LOW_TORQUE);

        SmartDashboard.putNumber("left enc", chassis.getPeriodicIO().leftPosition);
        SmartDashboard.putNumber("right enc", chassis.getPeriodicIO().rightPosition);
        SmartDashboard.putNumber("Pure heading", chassis.getPeriodicIO().heading.getDegrees());
        SmartDashboard.putNumber("Pure heading rad", chassis.getPeriodicIO().heading.getDegrees());

        SmartDashboard.putNumber("x pose", chassis.getPeriodicIO().x);
        SmartDashboard.putNumber("y pose", chassis.getPeriodicIO().y);
        SmartDashboard.putNumber("rotation", chassis.getPeriodicIO().rotation.getDegrees());
    }
}
