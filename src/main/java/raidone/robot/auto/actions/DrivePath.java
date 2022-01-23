// package raidone.robot.auto.actions;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import raidone.robot.Constants.ChassisConstants;
// import raidone.robot.Constants.AutoConstants;
// import raidone.robot.submodules.Chassis;
// import raidone.robot.submodules.ChassisController;

// public class DrivePath implements Action {
//     private static final Chassis chassis = Chassis.getInstance();
//     private static final ChassisController chassisController = ChassisController.getInstance();

//     private Trajectory path;

//     public DrivePath(Trajectory path) {
//         this.path = path;
//     }

//     /**
//      * Returns whether the action is finished or not.
//      * 
//      * @return if the action is finished
//      */
//     @Override
//     public boolean isFinished() {
//         return chassisController.isSettled();
//     }

//     /**
//      * Called AutoSequence iteratively until isFinished returns true. 
//      * Iterative logic lives in this method.
//      */
//     public void update() {

//     }

//     /**
//      * Runs code once when the action finishes, usually for clean up.
//      */
//     public void done() {
//         chassisController.zero();
//     }

//     /**
//      * Runs code once when the action is started for set up.
//      */
//     public void start() {}
// }
