package raidzero.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import raidzero.robot.auto.AutoRunner;
import raidzero.robot.submodules.SubmoduleManager;
// import raidzero.robot.submodules.OperationsManager;
import raidzero.robot.subsystems.Chassis;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    // private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    // private static final Teleop teleop = Teleop.getInstance();

    // private static final Superstructure superstructure = Superstructure.getInstance();

    // private AutoRunner autoRunner;
    private Chassis chassis;
    private XboxController master = new XboxController(0);

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        // Register all submodules here
        // submoduleManager.setSubmodules(
        //     superstructure
        // );
        // submoduleManager.onInit();

        // autoRunner = new AutoRunner();
        chassis = new Chassis();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // Stop autonomous
        // autoRunner.stop();
        // submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {
        // submoduleManager.onStart(Timer.getFPGATimestamp());

        // autoRunner.readSendableSequence();
        // autoRunner.start();
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        // autoRunner.onLoop(timestamp);
        // submoduleManager.onLoop(timestamp);
    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
        // Stop the autonomous
        // autoRunner.stop();

        // Start the teleop handler
        // teleop.onStart();
        // submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during teleop (50 Hz).
     */
    @Override
    public void teleopPeriodic() {
        // submoduleManager.onLoop(Timer.getFPGATimestamp());

        // Chassis control
        double throttle = master.getRawAxis(1); double turn = master.getRawAxis(3);
        boolean quickTurn = throttle == 0 && turn == 0;
        chassis.curvatureDrive(master.getRawAxis(1), master.getRawAxis(3), quickTurn);
    }

}
