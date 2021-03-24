package raidone.robot.submodules;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidone.robot.submodules.Limelight.LEDState;

public class Turret {
    //MOTOR & SENSOR INITIALIZATION
    public static final CANSparkMax turret = new CANSparkMax(61, MotorType.kBrushless);
    public static final CANEncoder turretEncoder = turret.getEncoder();

    public static enum Direction {
        LEFT, 
        RIGHT
    }

    //CONSTANTS
    private double kP, kD, prevError;

    //Constructor 
    public Turret(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
        prevError = 0.0;
    }

    //TURRET FUNCTIONS
    public void set(double percent){
        turret.setVoltage(percent * 12.0);
    }

    public void resetPID(){
        prevError = 0.0;
    }

    public void aim(Direction search){
        Limelight.setLED(LEDState.ON);
        Limelight.setPipeline(1);
        
        if(Limelight.targetDetected()){
            double error = Limelight.getX(); //might need to change to negative 
            double derivative = error - prevError;
            prevError = error;
            double output = error * kP + derivative * kD;
            set(output);
            SmartDashboard.putNumber("limelight error", error);
        } else {
            if(search == Direction.LEFT){
                set(1); //might need to change depending on which is left/right
            } else {
                set(-1);
            }
        }
    }

    public void reset(Direction dir){ //you better make sure to reset it in the correct place
        Limelight.setLED(LEDState.OFF);
        prevError = 0.0;

        if(dir == Direction.LEFT){    //otherwise u will actually be an idiot and the turret will die
            turret.set(0.75); //might need to change depending on which is left/right
        } else {
            turret.set(-0.75);
        }
    }
}