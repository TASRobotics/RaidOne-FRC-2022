package raidzero.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.Constants;
import raidzero.robot.wrappers.LazyTalonFX;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.LazyVictorSPX;

public class Lift {
    private LazyTalonFX leftLeader, rightFollower;

    public Lift() {
        leftLeader = new LazyTalonFX(Constants.LiftConstants.LEFT_LEADER_ID);
        configMotor(leftLeader, InvertType.None, NeutralMode.Brake);

        rightFollower = new LazyTalonFX(Constants.LiftConstants.RIGHT_FOLLOWER_ID);
        configMotor(rightFollower, InvertType.OpposeMaster, NeutralMode.Brake);
        rightFollower.follow(leftLeader);
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
     * Sets neutral brakeType of lift
     * 
     * @param brakeType coast or brake
     */
    public void setBrakeType(NeutralMode brakeType) {
        leftLeader.setNeutralMode(brakeType);
        rightFollower.setNeutralMode(brakeType);
    }

    /**
     * Moves lift 
     * 
     * @param val output 
     */
    public void move(ControlMode ctrl, double val) {
        leftLeader.set(ctrl, val);
    }
    
    /**
     * Controls lift movement via booleans
     * 
     * @param up moves the lift up
     * @param down moves the lift down
     */
    public void buttonControl(boolean up, boolean down) {
        int upVal = up ? 1 : 0;
        int downVal = down ? 1 : 0;
        move(ControlMode.PercentOutput, upVal - downVal);
    }

    /**
     * Controls lift movement via booleans
     * 
     * @param up moves the lift up at the desired speed
     * @param down moves the lift down at the desired speed
     * @param speed desired speed
     */
    public void buttonControl(boolean up, boolean down, double speed) {
        int upVal = up ? 1 : 0;
        int downVal = down ? 1 : 0;
        move(ControlMode.PercentOutput, (upVal - downVal) * speed);
    }
}
