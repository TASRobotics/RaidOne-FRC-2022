package raidone.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
    public static final class ChassisConstants {
        /** Motors */
        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_A_ID = 2;
        public static final int LEFT_FOLLOWER_B_ID = 3;

        public static final int RIGHT_LEADER_ID = 11;
        public static final int RIGHT_FOLLOWER_A_ID = 12;
        public static final int RIGHT_FOLLOWER_B_ID = 13;

        /** Sensors */
        public static final int IMU_ID = 0;

        /** Pneumatics */
        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final int SHIFTER_HIGH_TORQUE_ID = 7;
        public static final int SHIFTER_LOW_TORQUE_ID = 8;

        /** Velocity PID */
        public static final int PID_LOOP_IDX = 0;
        /** ROBORIO */
        // public static final double LEFT_kV = 0.222;
        // public static final double LEFT_kP = 0.045;
        // public static final double LEFT_kA = 3.78;
        // public static final double RIGHT_kV = 0.222;
        // public static final double RIGHT_kP = 0.045;
        // public static final double RIGHT_kA = 3.78;
        /** Motor Controller */
        public static final double LEFT_kV = 0.7;
        // public static final double LEFT_kP = 0.1;
        // public static final double LEFT_kA = 6.55;
        public static final double LEFT_kP = 0.0;
        public static final double LEFT_kA = 0.0;
        public static final double RIGHT_kV = 0.7;
        // public static final double RIGHT_kP = 0.1;
        // public static final double RIGHT_kA = 6.05;
        public static final double RIGHT_kP = 0.0;
        public static final double RIGHT_kA = 0.0;

        /** Drive kinematics (for ramsete) */
        public static final double TRACK_WIDTH = 0.69;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACK_WIDTH);
        
        /** base constants */

        /** // TODO 
         * 
         * - Fix the weird turning issue
         * - Add leader/follower shenanigans for climb 
         * - Characterize the chassis with Sysid (try to set max output to 1, 
         *   if that is impossible, remember to either set voltage to everything, 
         *   or divide everything by 12)
         * - Use the characterized values to test pathing, if it is somewhat 
         *   accurate, add ramsete to see if it works better
         */
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kP = 0.0;

        public static final int kEncoderCPR = 8129;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        /** Teleop Constants */
        public static final double MONOMIAL_SCALE = 0.0;
        public static final double SLEW_RATE_LIMIT = 1.75;
    }

    public static final class IntakeConstants {
        public static final int LEFT_LEADER_ID = 1;
        public static final int RIGHT_FOLLOWER_ID = 2;

        public static final int SOLENOID_DOWN_ID = 6;
        public static final int SOLENOID_UP_ID = 9;
    }

    public static final class EZClimbConstants {
        public static final int LEFT_ID = 21;
        public static final int RIGHT_ID = 22;

        public static final int SOLENOID_DOWN_ID = 5;
        public static final int SOLENOID_UP_ID = 10;
    }

    public static final class OIConstants {
        /** Controller constants */
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        /** Motion profile constants */
        public static final double kMaxSpeedMetersPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
        
        /** Ramsete constants */
        public static final double kRamseteB = 2;
        // public static final double kRamseteB = 3;
        public static final double kRamseteZeta = 0.7;
        // public static final double kRamseteZeta = 1.0;
    }

    /** Universal constants */
    public static final double DEADBAND = 0.06;
    public static final int TIMEOUT_MS = 10;
}
