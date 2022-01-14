package raidzero.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.LazyVictorSPX;
import raidzero.robot.wrappers.LazyTalonFX;

public class Chassis extends SubsystemBase {
    // MOTORS
    private LazyTalonSRX leftLeader, rightLeader;
    private LazyVictorSPX leftFollowerA, leftFollowerB, rightFollowerA, rightFollowerB;

    // VELOCITY
    private double prevVel;

    public Chassis() {
        leftLeader = new LazyTalonSRX(Constants.DriveConstants.LEFT_LEADER_ID);
        configMotor(leftLeader, InvertType.None, NeutralMode.Coast);
        rightLeader = new LazyTalonSRX(Constants.DriveConstants.RIGHT_LEADER_ID);
        configMotor(rightLeader, InvertType.InvertMotorOutput, NeutralMode.Coast);

        leftFollowerA = new LazyVictorSPX(Constants.DriveConstants.LEFT_FOLLOWER_A_ID);
        leftFollowerB = new LazyVictorSPX(Constants.DriveConstants.LEFT_FOLLOWER_B_ID);
        configMotor(leftFollowerA, InvertType.FollowMaster, NeutralMode.Coast);
        configMotor(leftFollowerB, InvertType.FollowMaster, NeutralMode.Coast);
        rightFollowerA = new LazyVictorSPX(Constants.DriveConstants.RIGHT_FOLLOWER_A_ID);
        rightFollowerB = new LazyVictorSPX(Constants.DriveConstants.RIGHT_FOLLOWER_B_ID);
        configMotor(rightFollowerA, InvertType.FollowMaster, NeutralMode.Coast);
        configMotor(rightFollowerB, InvertType.FollowMaster, NeutralMode.Coast);

        leftFollowerA.follow(leftLeader);
        leftFollowerB.follow(leftLeader);
        rightFollowerA.follow(rightLeader);
        rightFollowerB.follow(rightLeader);
    }

    /**
     * Configures a motor controller
     * 
     * @param motor motor controller to be configured
     * @param inversion whether the motor is reversed or not
     * @param brakeType motor brake type
     */
    private void configMotor(Object motor, InvertType inversion, NeutralMode brakeType) {
        if(motor instanceof LazyVictorSPX) {
            ((LazyVictorSPX)motor).configFactoryDefault();
            ((LazyVictorSPX)motor).setNeutralMode(brakeType);
            ((LazyVictorSPX)motor).setInverted(inversion);
        } else if(motor instanceof LazyTalonSRX) {
            ((LazyTalonSRX)motor).configFactoryDefault();
            ((LazyTalonSRX)motor).setNeutralMode(brakeType);
            ((LazyTalonSRX)motor).setInverted(inversion);
        } else if(motor instanceof LazyTalonFX) {
            ((LazyTalonFX)motor).configFactoryDefault();
            ((LazyTalonFX)motor).setNeutralMode(brakeType);
            ((LazyTalonFX)motor).setInverted(inversion);
        }
    }

    /**
     * Sets neutral brakeType of chassis
     * 
     * @param brakeType coast or brake
     */
    public void setBrakeType(NeutralMode brakeType) {
        leftLeader.setNeutralMode(brakeType);
        leftFollowerA.setNeutralMode(brakeType);
        leftFollowerB.setNeutralMode(brakeType);

        rightLeader.setNeutralMode(brakeType);
        rightFollowerA.setNeutralMode(brakeType);
        rightFollowerB.setNeutralMode(brakeType);
    }

    /**
     * Moves chassis 
     * 
     * @param left left setting 
     * @param right right setting
     */
    public void move(ControlMode ctrl, double left, double right) {
        leftLeader.set(ctrl, left);
        rightLeader.set(ctrl, right);
    }

    /**
     * Tank drive for opcontrol: 
     * - Left joystick controls left chassis movement
     * - Right joystick controls right chassis movement
     * 
     * @param leftIn left joystick input [-1, 1]
     * @param rightIn right joystick input [-1, 1]
     */
    public void tankDrive(double leftIn, double rightIn) {
        leftIn = deadband(leftIn, Constants.JOYSTICK_DEADBAND);
        rightIn = deadband(rightIn, Constants.JOYSTICK_DEADBAND);
        move(ControlMode.PercentOutput, leftIn, rightIn);
    }

    /**
     * Arcade drive for opcontrol: 
     * - One joystick controls throttle
     * - One joystick controls turn
     * 
     * @param throttle throttle joystick input [-1, 1]
     * @param turn turn joystick input [-1, 1]
     */
    public void arcadeDrive(double throttle, double turn) {
        throttle = deadband(throttle, Constants.JOYSTICK_DEADBAND);
        turn = deadband(turn, Constants.JOYSTICK_DEADBAND);
        move(ControlMode.PercentOutput, throttle + turn, throttle - turn);
    }

    /**
     * Curvature drive - drives like an actual car (also the superior drive type)
     * 
     * @param throttle throttle joystick input [-1, 1]
     * @param turn turn joystick input [-1, 1]
     * @param quickTurn point turns if true
     */
    public void curvatureDrive(double throttle, double turn, boolean quickTurn) {
        throttle = deadband(throttle, Constants.JOYSTICK_DEADBAND);
        turn = deadband(turn, Constants.JOYSTICK_DEADBAND);

        // Compute velocity, right stick = curvature if no quickturn, else power
        double leftSpeed = throttle + (quickTurn ? turn : Math.abs(throttle) * turn);
        double rightSpeed = throttle - (quickTurn ? turn : Math.abs(throttle) * turn);

        // Normalize velocity
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }
        move(ControlMode.PercentOutput, leftSpeed, rightSpeed);
    }

    public double deadband(double val, double minRange) {
        return Math.abs(val) < minRange ? 0 : val;
    }

    public double getLinearVelocity(LazyTalonSRX motor) {
        double vel = 0.0;
        prevVel = motor.getSelectedSensorVelocity();
        // do some gear ratio shenanigans
        return vel;
    }

    public void moveLinearVelocity(double vel) {
        double rpm = 0.0;
        // vel to rpm conversion shenanigans (or wherever the encoder is located)
        // hmm, probably only applicable if we use talon fx's
    }

    public void setCurrentLimiter() {}
}
