package raidone.robot.submodules;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import raidone.robot.submodules.Limelight.LEDState;

class Turret {
    //MOTOR & SENSOR INITIALIZATION
    public static final CANSparkMax turret = new CANSparkMax(61, MotorType.kBrushless);
    public static final CANEncoder turretEncoder = turret.getEncoder();

    public enum Direction {
        LEFT, 
        RIGHT
    }

    //CONSTANTS
    private double kP, kD, kF, prevError, searchSpeed;

    //Constructor 
    Turret(double kP, double kD, double kF, double searchSpeed){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        prevError = 0.0;
        this.searchSpeed = searchSpeed;
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
        
        if(Limelight.targetDetected()){
            double error = Limelight.getX(); //might need to change to negative 
            double derivative = error - prevError;
            prevError = error;
            double output = error * kP + derivative * kD;
            set(output);
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

        if(dir == Direction.LEFT){    //otherwise u will actually be an idiot and the turret will die
            turret.set(1); //might need to change depending on which is left/right
        } else {
            turret.set(-1);
        }
    }
}