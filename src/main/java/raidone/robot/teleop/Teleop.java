package raidone.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.Constants;
import raidone.robot.submodules.Chassis;
import raidone.robot.submodules.EZClimb;
import raidone.robot.submodules.Intake;
import raidone.robot.submodules.Chassis.GearShift;
import raidone.robot.submodules.EZClimb.EZClimbState;
import raidone.robot.submodules.Intake.IntakeState;

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
    private XboxController partner = new XboxController(1);

    private static Chassis chassis = Chassis.getInstance();
    private static Intake intake = Intake.getInstance();
    private static EZClimb climb = EZClimb.getInstance();

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        chassis.changeShifterState(GearShift.LOW_TORQUE);

        chassis.zero();
    }

    /**
     * Continuously loops in teleop.
     */
    boolean shiftState = false, prevShiftState = false, driveState = false, prevDriveState = false, switchFront = false, prevSwitchFront = false;
    // int drive = 0;
    boolean shift = false;
    int val = 1;
    // String message = "";
    public void onLoop() {
        double leftY = master.getLeftY() * val * 0.9;
        switchFront = master.getRightStickButton();
        if(switchFront && !prevSwitchFront) {
            val *= -1;
        }
        prevSwitchFront = switchFront;
        chassis.curvatureDrive(leftY, -master.getRightX(), Math.abs(master.getLeftY()) < Constants.DEADBAND);
        // chassis.tankDrive(master.getLeftY(), master.getRightY());
        

        shiftState = master.getLeftBumper() || partner.getAButton();
        if(shiftState && !prevShiftState) {
            shift = !shift;
        }
        if(shift) {
            chassis.changeShifterState(GearShift.LOW_TORQUE);
        } else {
            chassis.changeShifterState(GearShift.HIGH_TORQUE);
        }
        prevShiftState = shiftState;
        SmartDashboard.putString("Shift state", shift ? "low torque" : "high torque");

        // driveState = master.getAButton();
        // if(driveState && !prevDriveState) {
        //     drive++;
        // }
        // switch(drive %= 3) {
        //     case 0: 
        //         chassis.curvatureDrive(-master.getRawAxis(1), master.getRawAxis(4), Math.abs(master.getRawAxis(1)) < Constants.DEADBAND);
        //         message = "Curvature Drive";
        //         break;
        //     case 1:
        //         chassis.tankDrive(-master.getRawAxis(1), -master.getRawAxis(5));
        //         message = "Tank Drive";
        //         break;
        //     case 2:
        //         chassis.arcadeDrive(-master.getRawAxis(1), master.getRawAxis(4));
        //         message = "Arcade Drive";
        //         break;
        // }
        // prevDriveState = driveState;
        // SmartDashboard.putString("Drive type", message);

        SmartDashboard.putNumber("x pose", chassis.getPeriodicIO().x);
        SmartDashboard.putNumber("y pose", chassis.getPeriodicIO().y);
        SmartDashboard.putNumber("rotation", chassis.getPeriodicIO().rotation.getDegrees());

        if(master.getStartButton()) {
            climb.setState(EZClimbState.UP);
        } else {
            climb.setState(EZClimbState.DOWN);
        }

        /** Shift */
        if(master.getRightBumper()) {
            climb.setSpeed(master.getLeftTriggerAxis());
        } else {
            intake.autoSet(master.getRightTriggerAxis() - master.getLeftTriggerAxis());
        }
        intake.setState(partner.getLeftBumper() || partner.getRightBumper() ? IntakeState.DOWN : IntakeState.UP);
        intake.setPercentSpeed(partner.getRightTriggerAxis() - partner.getLeftTriggerAxis());
    }
}
