package raidone.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidone.robot.auto.AutoRunner;
import raidone.robot.Constants.*;
import raidone.robot.teleop.Teleop;
import raidone.robot.submodules.Drive;
import raidone.robot.submodules.SubmoduleManager;
import raidone.robot.submodules.Flywheel;
import raidone.robot.submodules.Throat;
import raidone.robot.submodules.Turret;
// import raidone.robot.submodules.Limelight;
// import raidone.robot.submodules.Intake;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private static final Teleop teleop = Teleop.getInstance();

    private static final Drive moduleDrive = Drive.getInstance();
    public static final Flywheel flywheel = new Flywheel(0.5, 0.00006, 0.0005);
    public static final Turret turret = new Turret(TurretConstants.TURRET_KP, TurretConstants.TURRET_KD);//0.15
    public static final Throat throat = new Throat();

    private AutoRunner autoRunner;

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        // Register all submodules here
        submoduleManager.setSubmodules(
            moduleDrive
            // moduleShooter,
            // moduleIntake,
            // moduleLimelight,
            // moduleThroat
        );
        submoduleManager.onInit();

        autoRunner = new AutoRunner();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // Stop autonomous
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {
        submoduleManager.onStart(Timer.getFPGATimestamp());

        autoRunner.readSendableSequence();
        autoRunner.start();
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        autoRunner.onLoop(timestamp);
        submoduleManager.onLoop(timestamp);
    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
        // Stop the autonomous
        autoRunner.stop();

        // Start the teleop handler
        teleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during teleop (50 Hz).
     */
    @Override
    public void teleopPeriodic() {
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }

}
