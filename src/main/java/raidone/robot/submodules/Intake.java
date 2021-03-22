package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

class Intake {
    public static final TalonFX intake = new TalonFX(31);

    public static void set(double percent){
        intake.set(TalonFXControlMode.PercentOutput, percent);
    }
}