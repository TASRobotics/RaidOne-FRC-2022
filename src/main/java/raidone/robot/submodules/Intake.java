package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake {

    public static TalonFX intake;

    // TalonID = 31
    public Intake(int talonID){
        intake = new TalonFX(talonID);
    }
    
    public void set(double percent){
        intake.set(TalonFXControlMode.PercentOutput, percent);
    }
}