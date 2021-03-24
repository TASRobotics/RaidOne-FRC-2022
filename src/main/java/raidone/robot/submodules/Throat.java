package raidone.robot.submodules;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

public class Throat {
    //THROAT MOTORS
    public static final WPI_TalonFX _throatright = new WPI_TalonFX(21);
    public static final WPI_TalonFX _throatleft = new WPI_TalonFX(22);

    //THROAT SENSOR(S)
    public static final TimeOfFlight _throatSensor = new TimeOfFlight(0);

    public enum Speed { 
        DOWN, 
        UP, 
        UP_SLOW, 
        DOWN_SLOW, 
        STOP
    }

    //CONSTANTS
    public static final double BALL_DETECTED_VALUE = 80.0;

    //HELPER FUNCTIONS
    public static void set(double percent){
        _throatright.set(percent);
    }

    public static void setState(Speed set){
        if(set == Speed.DOWN){
            set(-1);
        }
        if(set == Speed.UP){
            set(1);
        }
        if(set == Speed.UP_SLOW){
            set(0.3);
        }
        if(set == Speed.DOWN_SLOW){
            set(-0.3);
        } 
        if(set == Speed.STOP){
            set(0);
        }
    }

    public static void index(){
        set(0.20); //i don't even think this is wired
        if(_throatSensor.getRange() < BALL_DETECTED_VALUE){ //idk, tune or smth
            set(0);
        }
    }
}
