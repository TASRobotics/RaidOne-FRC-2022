package raidone.robot.teleop;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import raidone.robot.Constants.DriveConstants;
import raidone.robot.Constants.IntakeConstants;
import raidone.robot.auto.actions.DebugLimelightDistance;
import raidone.robot.Robot;
// import raidone.robot.auto.actions.DebugLimelightDistance;
import raidone.robot.dashboard.Tab;
import raidone.robot.submodules.Angler;
import raidone.robot.submodules.Drive;
import raidone.robot.submodules.Shooter;
import raidone.robot.submodules.Throat;
import raidone.robot.submodules.Intake;
import raidone.robot.submodules.Drive.GearShift;
import raidone.robot.submodules.Turret;
import raidone.robot.submodules.Flywheel;
import raidone.robot.utils.JoystickUtils;

public class Teleop {

    private enum DriveMode {
        TANK(0), ARCADE(1), CURVATURE(2);

        private static final DriveMode[] modes = { TANK, ARCADE, CURVATURE };
        public final int index;

        private DriveMode(int index) {
            this.index = index;
        }

        private DriveMode next() {
            if (index == modes.length - 1) {
                return modes[0];
            }
            return modes[index + 1];
        }
    }

    private static Teleop instance = null;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private Teleop() {
    }

    private static Drive drive = Drive.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Throat throat = Throat.getInstance();

    private XboxController controller = new XboxController(0);

    // private DebugLimelightDistance debugDistance = new DebugLimelightDistance();

    private DriveMode driveMode = DriveMode.TANK;

    private NetworkTableEntry driveModeEntry = Shuffleboard.getTab(Tab.MAIN).add("Drive Mode", driveMode.toString())
            .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(2, 2).getEntry();

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        drive.zero();
        drive.stop();
        drive.setGearShift(GearShift.LOW);
        drive.setBrakeMode(true);

        debugDistance.start();
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {

        p1Loop();
        // p2Loop();

        // debugDistance.update();
    }

    private void p1Loop() {
        //
        // REGARDLESS OF HYPERSHIFT
        //
        // Reversing analog to digital
        boolean reverse = JoystickUtils.deadband(controller.getTriggerAxis(Hand.kRight)) != 0;

        /**
         * Drivetrain
         */
        // Cycle through drive modes
        if (controller.getBackButtonPressed()) {
            driveMode = driveMode.next();
        }
        driveModeEntry.setString(driveMode.toString());

        switch (driveMode) {
            case TANK:
                drive.tank(
                        JoystickUtils.monomialScale(JoystickUtils.deadband(-controller.getY(Hand.kLeft)),
                                DriveConstants.JOYSTICK_EXPONENT, DriveConstants.JOYSTICK_COEFFICIENT),
                        JoystickUtils.monomialScale(JoystickUtils.deadband(-controller.getY(Hand.kRight)),
                                DriveConstants.JOYSTICK_EXPONENT, DriveConstants.JOYSTICK_COEFFICIENT),
                        reverse);
                break;
            case ARCADE:
                drive.arcade(JoystickUtils.deadband(-controller.getY(Hand.kLeft)),
                        JoystickUtils.deadband(controller.getX(Hand.kRight)), reverse);
                break;
            case CURVATURE:
                double xSpeed = JoystickUtils.deadband(-controller.getY(Hand.kLeft));
                drive.curvatureDrive(xSpeed, JoystickUtils.deadband(controller.getX(Hand.kRight)),
                        Math.abs(xSpeed) < 0.1 // TODO:
                // Change
                // quick
                // turn
                );  
                break;
        }

        // Braking
        if (controller.getAButtonPressed()) {
            drive.setBrakeMode(true);
        } else if (controller.getAButtonReleased()) {
            drive.setBrakeMode(false);
        }

        if(controller.getXButton()){
            Intake.set(0.4);
        }
        else {
            Intake.set(0);
        }

        //
        // WITHOUT HYPERSHIFT
        //
        // if (!controller.getBumper(Hand.kRight)) {

            if(controller.getBumperPressed(Hand.kLeft)){
                Throat.set(0.5);
            } else {
                Throat.index();
            }

            if (controller.getBumper(Hand.kRight)){
                Robot.flywheel.setVel(0.8);
                Robot.turret.aim(Turret.Direction.RIGHT);
            } else if(controller.getYButtonPressed()){
                Robot.turret.set(1);
            }
            else {
                Robot.turret.resetPID();
                Robot.turret.set(0);
                Robot.flywheel.set(0);
            }
            Angler.setPower(controller.getTriggerAxis(Hand.kLeft) - controller.getTriggerAxis(Hand.kRight));
        // }

        //  
        // WITH HYPERSHIFT
        //
        /**
         * Intake
         */
        //Run intake out
        // intake.intakeBalls(JoystickUtils
        //         .deadband(IntakeConstants.CONTROL_SCALING_FACTOR * (-controller.getTriggerAxis(Hand.kLeft))));

        /**
         * Throat
         */
        // Loop down throat
        // if (controller.getBumperPressed(Hand.kLeft)) {
        //     throat.loopBalls(-70);
        // } else if (controller.getBumperReleased(Hand.kLeft)) {
        //     throat.loopBalls(0);
        // }

    }
}
