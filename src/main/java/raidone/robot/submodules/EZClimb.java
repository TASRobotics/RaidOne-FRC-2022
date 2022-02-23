package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;

import raidone.robot.Constants.EZClimbConstants;
import raidone.robot.submodules.Chassis.PeriodicIO;
import raidone.robot.wrappers.LazyTalonFX;

public class EZClimb extends Submodule {
    public static class PeriodicIO {
        public double desiredSpeed = 0.0;
    }

    /** Motors */
    private static final LazyTalonFX mLeftLeader = new LazyTalonFX(EZClimbConstants.LEFT_LEADER_ID);
    private static final LazyTalonFX mRightFollower = new LazyTalonFX(EZClimbConstants.RIGHT_FOLLOWER_ID);

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
        mLeftLeader.configFactoryDefault();
        mRightFollower.configFactoryDefault();

        mRightFollower.follow(mLeftLeader);

        mLeftLeader.setInverted(InvertType.None);
        mRightFollower.setInverted(InvertType.OpposeMaster);
    }

    @Override
    public void onStart(double timestamp) {
        periodicIO = new PeriodicIO();

        stop();
    }

    @Override
    public void run() {
        mLeftLeader.set(ControlMode.PercentOutput, periodicIO.desiredSpeed);
    }

    @Override
    public void stop() {
        periodicIO.desiredSpeed = 0.0;
        mLeftLeader.set(ControlMode.Disabled, 0.0);
    }

    public void setSpeeed(double speed) {
        periodicIO.desiredSpeed = speed;
    }
}
