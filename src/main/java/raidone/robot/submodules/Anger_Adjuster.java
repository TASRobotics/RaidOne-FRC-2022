package raidone.robot.submodules;

import edu.wpi.first.wpilibj.AnalogInput;
//My bad attempt at using servos
import edu.wpi.first.wpilibj.Servo;

class Angler {
    //ANGLE ADJUSTERS 
    public static final Servo _rightServo = new Servo(0);
    public static final Servo _leftServo = new Servo(1);

    //ANGLE SENSOR
    public static final AnalogInput _hoodPotentiometer = new AnalogInput(1); //imagine not having a pot

    //CONSTANTS 
    double kP, kD, kF, prevError;

    //Constructor 
    Angler(double kP, double kD, double kF){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
    }

    //ANGLE ADJUSTER HELPER FUNCTIONS
    public static void setPower(double percent){
        _leftServo.set(percent);
        _rightServo.set(-percent);
    }
}