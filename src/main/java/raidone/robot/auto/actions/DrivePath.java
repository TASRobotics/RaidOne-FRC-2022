package raidone.robot.auto.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import raidone.robot.Constants.ChassisConstants;
import raidone.robot.Constants.AutoConstants;
import raidone.robot.submodules.Chassis;

public class DrivePath {
    private static final Chassis chassis = Chassis.getInstance();

    private DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private TrajectoryConfig config;
    private RamseteCommand ramsete;
    
    private Trajectory path;

    public DrivePath(Trajectory path) {
        this.path = path;

        autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    ChassisConstants.ksVolts,
                    ChassisConstants.kvVoltSecondsPerMeter,
                    ChassisConstants.kaVoltSecondsSquaredPerMeter),
                ChassisConstants.kDriveKinematics,
                10);

        config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(ChassisConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        ramsete =
            new RamseteCommand(
                this.path,
                chassis::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    ChassisConstants.ksVolts,
                    ChassisConstants.kvVoltSecondsPerMeter,
                    ChassisConstants.kaVoltSecondsSquaredPerMeter),
                ChassisConstants.kDriveKinematics,
                chassis::getWheelSpeeds,
                new PIDController(ChassisConstants.kPDriveVel, 0, 0),
                new PIDController(ChassisConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                chassis::tankDriveVolts,
                chassis);
    }

    public boolean isFinished() {
        return ramsete.isFinished();
    }

    public void execute() {
        ramsete.execute();
    }
}
