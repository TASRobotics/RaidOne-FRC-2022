package raidone.robot.submodules;

//Potentiometer
import edu.wpi.first.wpilibj.AnalogInput;
//Servo
import edu.wpi.first.wpilibj.Servo;

public class Angler {
    //ANGLE ADJUSTERS 
    // Right: 0, Left: 1
    public static Servo _rightServo;
    public static Servo _leftServo;

    //ANGLE SENSOR
    public static final AnalogInput _hoodPot = new AnalogInput(0); 

    //CONSTANTS 
    double kP, kD, prevError;

    //Constructor 
    public Angler(int rightServoID, int leftServoID, double kP, double kD){
        _rightServo = new Servo(rightServoID);
        _leftServo = new Servo(leftServoID);
        this.kP = kP;
        this.kD = kD;
        this.prevError = 0;
    }

    //ANGLE ADJUSTER HELPER FUNCTIONS
    public double map(double value, double start1, double stop1, double start2, double stop2){
        return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
    }

    public void setPower(double percent){
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