package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidone.robot.Constants.EZClimbConstants;
import raidone.robot.wrappers.InactiveDoubleSolenoid;
import raidone.robot.wrappers.LazyTalonFX;

public class EZClimb extends Submodule {
    public static class PeriodicIO {
        public double desiredSpeed = 0.0;
    }

    public static enum EZClimbState {
        UP, OFF, DOWN;
    }

    /** Motors */
    private static final LazyTalonFX mLeft = new LazyTalonFX(EZClimbConstants.LEFT_ID);
    private static final LazyTalonFX mRight = new LazyTalonFX(EZClimbConstants.RIGHT_ID);

    /** Pneumatics */
    private static final InactiveDoubleSolenoid solenoid = new InactiveDoubleSolenoid(
        EZClimbConstants.SOLENOID_DOWN_ID, EZClimbConstants.SOLENOID_UP_ID);

    private PeriodicIO periodicIO = new PeriodicIO();

    private EZClimb() {}
    private static EZClimb instance = null;
    public static EZClimb getInstance() {
        if(instance == null) {
            instance = new EZClimb();
        }
        return instance;
    }

    @Override
    public void onInit() {
        mLeft.configFactoryDefault();
        mRight.configFactoryDefault();

        // mRightFollower.follow(mLeftLeader);

        mLeft.setInverted(false);
        mRight.setInverted(true);

        mLeft.setNeutralMode(NeutralMode.Coast);
        mRight.setNeutralMode(NeutralMode.Coast);
        // mRightFollower.setInverted(InvertType.OpposeMaster);
    }

    @Override
    public void onStart(double timestamp) {
        periodicIO = new PeriodicIO();

        stop();
    }

    @Override
    public void run() {
        mLeft.set(ControlMode.PercentOutput, periodicIO.desiredSpeed);
        mRight.set(ControlMode.PercentOutput, periodicIO.desiredSpeed);
        // mRightFollower.set(ControlMode.PercentOutput, periodicIO.desiredSpeed);
    }

    @Override
    public void stop() {
        periodicIO.desiredSpeed = 0.0;
        mLeft.set(ControlMode.Disabled, 0.0);
        mRight.set(ControlMode.Disabled, 0.0);
    }

    /**
     * Sets the speed of EZClimb - it is physically 
     * impossible to turn it the other way
     * 
     * @param speed desired speed
     */
    public void setSpeed(double speed) {
        periodicIO.desiredSpeed = -Math.abs(speed);
    }

    /**
     * Sets the deploy state of the EZClimb
     * 
     * @param state deploy state
     */
    public void setState(EZClimbState state) {
        switch(state) {
            case UP:
                solenoid.set(InactiveDoubleSolenoid.Value.kForward);
                break;

            case OFF:
                solenoid.set(InactiveDoubleSolenoid.Value.kOff);
                break;

            case DOWN:
                solenoid.set(InactiveDoubleSolenoid.Value.kReverse);
                break;
        }
    }
}
