package raidone.robot.submodules;

//Potentiometer
import edu.wpi.first.wpilibj.AnalogInput;
//Servo
import edu.wpi.first.wpilibj.Servo;

public class Angler {
    //ANGLE ADJUSTERS 
    public static final Servo _rightServo = new Servo(0);
    public static final Servo _leftServo = new Servo(1);

    //ANGLE SENSOR
    public static final AnalogInput _hoodPot = new AnalogInput(0); //imagine not having a pot

    //CONSTANTS 
    double kP, kD, prevError;

    //Constructor 
    Angler(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
        this.prevError = 0;
    }

    //ANGLE ADJUSTER HELPER FUNCTIONS
    public static double map(double value, double start1, double stop1, double start2, double stop2){
        return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
    }

    public static void setPower(double percent){
        //_leftServo.set(percent);
        //_rightServo.set(-percent); //reversed

        double leftSpeed = map(percent, -1, 1, 0, 180);
        double rightSpeed = map(percent, -1, 1, 180, 0);
        _leftServo.setAngle(leftSpeed);
        _rightServo.setAngle(rightSpeed);
    }
    

    public void moveTo(double position){
        double error = position - _hoodPot.getValue();
        double derivative = error - prevError;
        prevError = error;
        double output = error * kP + derivative * kD;
        setPower(output);
    }
}