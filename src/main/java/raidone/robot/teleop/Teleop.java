package raidone.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.Constants;
import raidone.robot.submodules.Chassis;
<<<<<<< HEAD
import raidone.robot.submodules.Intake;
=======
>>>>>>> aae4a974c2517aafe6af7ff8edec645e0427f4a6
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
<<<<<<< HEAD
    private static Intake intake = Intake.getInstance();
=======
>>>>>>> aae4a974c2517aafe6af7ff8edec645e0427f4a6

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
    boolean shiftState = false, prevShiftState = false, driveState = false, prevDriveState = false;
    int drive = 0;
    boolean shift = false;
    String message = "";
    public void onLoop() {
        chassis.curvatureDrive(-master.getRawAxis(1), master.getRawAxis(2), Math.abs(master.getRawAxis(1)) < Constants.DEADBAND);
        shiftState = master.getLeftBumper();
        if(shiftState && !prevShiftState) {
            shift = !shift;
        }
        if(shift) {
            chassis.changeShifterState(GearShift.LOW_TORQUE);
        } else {
            chassis.changeShifterState(GearShift.HIGH_TORQUE);
        }
        prevShiftState = shiftState;
        SmartDashboard.putBoolean("Shift state", shift);

        driveState = master.getAButton();
        if(driveState && !prevDriveState) {
            drive++;
        }
        switch(drive %= 3) {
            case 0: 
                chassis.curvatureDrive(-master.getRawAxis(1), master.getRawAxis(4), Math.abs(master.getRawAxis(1)) < Constants.DEADBAND);
                message = "Curvature Drive";
                break;
            case 1:
                chassis.tankDrive(-master.getRawAxis(1), -master.getRawAxis(5));
                message = "Tank Drive";
                break;
            case 2:
                chassis.arcadeDrive(-master.getRawAxis(1), master.getRawAxis(4));
                message = "Arcade Drive";
                break;
        }
        prevDriveState = driveState;
        SmartDashboard.putString("Drive type", message);

        SmartDashboard.putNumber("x pose", chassis.getPeriodicIO().x);
        SmartDashboard.putNumber("y pose", chassis.getPeriodicIO().y);
        SmartDashboard.putNumber("rotation", chassis.getPeriodicIO().rotation.getDegrees());
    }
}
