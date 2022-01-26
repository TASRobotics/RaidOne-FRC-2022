package raidone.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.Constants;
import raidone.robot.submodules.Chassis;
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
    // private static ChassisController chassisController = ChassisController.getInstance();

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
    public void onLoop() {
        chassis.curvatureDrive(-master.getRawAxis(1), master.getRawAxis(2), Math.abs(master.getRawAxis(1)) < Constants.DEADBAND);
        if(master.getLeftBumper()) chassis.changeShifterState(GearShift.HIGH_TORQUE);
        else if(master.getRightBumper()) chassis.changeShifterState(GearShift.LOW_TORQUE);

        // chassis.updateOdometry();

        SmartDashboard.putNumber("left enc", chassis.getLeftEncoderDistance());
        SmartDashboard.putNumber("right enc", chassis.getLeftEncoderDistance());
        SmartDashboard.putNumber("Pure heading", chassis.getHeading());
        SmartDashboard.putNumber("Pure heading rad", Math.toRadians(chassis.getHeading()));

        SmartDashboard.putNumber("x pose", chassis.getPose().getX());
        SmartDashboard.putNumber("y pose", chassis.getPose().getY());
        SmartDashboard.putNumber("rotation", chassis.getPose().getRotation().getDegrees());
    }
}
