package raidone.robot.submodules;

import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Throat {
    //THROAT MOTORS
    public static final WPI_TalonFX _throatright = new WPI_TalonFX(21);
    public static final WPI_TalonFX _throatleft = new WPI_TalonFX(22);

    //THROAT SENSOR(S)
    public static final AnalogInput _ballDetector = new AnalogInput(1); //change

    public enum Speed { 
        DOWN, 
        UP, 
        UP_SLOW, 
        DOWN_SLOW, 
        STOP
    }

    //CONSTANTS
    public static final double BALL_DETECTED_VALUE = 0.0;

    //HELPER FUNCTIONS
    public static void set(double percent){
        _throatright.set(percent);
    }

    public static void setState(Speed speed){
        switch(speed.ordinal()){ //is this bad programming practice? yes. but am i gonna use it? yes.
            case 0:
            set(-1);
            break;

            case 1:
            set(1);
            break;

            case 2:
            set(0.25);
            break;

            case 3:
            set(-0.25);
            break;

            default:
            set(0);
            break;
        }
    }

    public static void index(){
        set(1); //i don't even think this is wired
        if(_ballDetector.getValue() > BALL_DETECTED_VALUE){ //idk, tune or smth
            set(0);
        }
    }
}
