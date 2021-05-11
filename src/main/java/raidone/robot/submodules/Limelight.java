package raidone.robot.submodules;

//Limelight 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static final NetworkTableEntry tv = table.getEntry("tv");
    public static final NetworkTableEntry tx = table.getEntry("tx");
    public static final NetworkTableEntry ty = table.getEntry("ty");
    public static final NetworkTableEntry ta = table.getEntry("ta");
    public static final NetworkTableEntry ledMode = table.getEntry("ledMode");
    public static final NetworkTableEntry pipeline = table.getEntry("pipeline");

    public enum LEDState {
        NORMAL,
        OFF, 
        BLINK, 
        ON
    }

    public enum Pipeline {
        FAR_BALL,
        TARGET
    }

    //FIELD CONSTANTS
    public static final double GOAL_HEIGHT = 98.25; //inches (249.6 cm, 8ft & 2.25in)
    public static final double LIMELIGHT_ANGLE = Math.toRadians(30.0);
    public static final double LIMELIGHT_HEIGHT = 27.2; //inches

    //LIMELIGHT HELPER FUNCTIONS
    public static void setLED(LEDState state){
        ledMode.setNumber(state.ordinal());
    }

    public static void setPipeline(int number){
        pipeline.setNumber(number);
    }

    public static boolean targetDetected(){
        if(tv.getDouble(0.0) > 0.0){
            return true;
        } return false;
    }

    public static double getX(){
        return tx.getDouble(0.0);
    }

    public static double getY(){
        return ty.getDouble(0.0);
    }

    public static double getA(){
        return ta.getDouble(0.0);
    }

    public static double getDistance(){
        return (GOAL_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(getY())); 
    }
}