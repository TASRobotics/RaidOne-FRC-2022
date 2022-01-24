// package raidone.robot.submodules;

// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.trajectory.Trajectory;

// import raidone.robot.Constants.ChassisConstants;
// import raidone.robot.Constants.AutoConstants;

// public class ChassisController extends Submodule {
//     private Chassis chassis = Chassis.getInstance();

    
//     private DifferentialDriveWheelSpeeds wheelSpeeds;

//     private Trajectory path;

//     private static ChassisController instance = null;
//     public static ChassisController getInstance() {
//         if(instance == null) {
//             instance = new ChassisController();
//         }
//         return instance;
//     }

//     private boolean start;
//     private double startTime;

//     /** Runs when it gets initialized */
//     @Override 
//     public void onInit() {
//         /** Odom init */
        
        
//         /** Ramsete init */
        
//     }

//     /** Runs at the start of auton/teleop */
//     @Override
//     public void onStart(double timestamp) {
//         // zero();
//     }

//     /** Loops */
//     @Override
//     public void run() {
//         // Runs odometry in the background
        

//         // chassis.setDesiredVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
//     }

//     /** Loops & gives timestamp updates */
//     @Override
//     public void update(double timestamp) {
//         // if(!start) {
//         //     this.startTime = timestamp;
//         //     start = true;
//         // }
//         // double relativeTime = timestamp - startTime;
//         // ChassisSpeeds speeds = ramseteController.calculate(getPose(), path.sample(relativeTime));
//         // wheelSpeeds = ChassisConstants.DRIVE_KINEMATICS.toWheelSpeeds(speeds);
//     }

//     /** Stops chassis */
//     @Override
//     public void stop() {
//         chassis.stop();
//     }

//     /** Zeros position */
//     @Override
//     public void zero() {
//         chassis.zero();
        
//     }

//     public boolean isSettled() {
//         return ramseteController.atReference();
//     }

//     public void setTolerance(Pose2d pose) {
//         ramseteController.setTolerance(pose);
//     }

//     /** Sets path */
//     public void setPath(Trajectory path) {
//         this.path = path;
//     }

    
// }
