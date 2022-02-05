package raidone.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
    public static final class ChassisConstants {
        /** Motors */
        public static final int LEFT_LEADER_ID = 11;
        public static final int LEFT_FOLLOWER_A_ID = 12;
        public static final int LEFT_FOLLOWER_B_ID = 13;

        public static final int RIGHT_LEADER_ID = 1;
        public static final int RIGHT_FOLLOWER_A_ID = 2;
        public static final int RIGHT_FOLLOWER_B_ID = 3;

        /** Sensors */
        public static final int IMU_ID = 0;

        /** Pneumatics */
        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SHIFTER_HIGH_TORQUE_ID = 0;
        public static final int SHIFTER_LOW_TORQUE_ID = 1;

        /** Velocity PID */
        public static final int PID_LOOP_IDX = 0;
        public static final double LEFT_kV = 0.222;
        public static final double LEFT_kP = 0.045;
        public static final double LEFT_kA = 3.78;
        public static final double RIGHT_kV = 0.222;
        public static final double RIGHT_kP = 0.045;
        public static final double RIGHT_kA = 3.78;

        /** Drive kinematics (for ramsete) */
        public static final double TRACK_WIDTH = 0.69;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACK_WIDTH);
        
        /** base constants */
        public static final int kEncoderCPR = 8129;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.93485;
        public static final double kvVoltSecondsPerMeter = 2.2499;
        public static final double kaVoltSecondsSquaredPerMeter = 0.83689;
        
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 3.5955;
    }

    public static final class OIConstants {
        /** Controller constants */
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        /** Motion profile constants */
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        
        /** Ramsete constants */
        // public static final double kRamseteB = 2;
        public static final double kRamseteB = 3;
        // public static final double kRamseteZeta = 0.7;
        public static final double kRamseteZeta = 1.0;
    }

    /** Universal constants */
    public static final double DEADBAND = 0.06;
    public static final int TIMEOUT_MS = 10;
}
