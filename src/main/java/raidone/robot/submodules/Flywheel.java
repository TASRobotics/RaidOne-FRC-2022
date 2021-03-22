package raidone.robot.submodules;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Talon FX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
//Sparkmax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Flywheel {
    //MAIN FLYWHEEL
    public static final WPI_TalonFX _shooterleft = new WPI_TalonFX(42);
    public static final WPI_TalonFX _shooterright = new WPI_TalonFX(41);
    public static final TalonFXConfiguration config = new TalonFXConfiguration();

    //KICKER WHEEL
    public static final CANSparkMax _kickerleft = new CANSparkMax(52, MotorType.kBrushless);
    public static final CANSparkMax _kickerright = new CANSparkMax(51, MotorType.kBrushless);
    public static final CANEncoder _kickerEncoder = _kickerright.getEncoder();
    public static final CANPIDController sparkmaxPID = _kickerright.getPIDController();
    //spark maxs r actually garbage

    public enum velocityControl {
        PID, 
        TBH
    }

    //CONSTANTS
    public static final double KICKER_FREE_SPEED = 11000; //RPM
    public static final double MAIN_FREE_SPEED = 6380.0; //RPM

    //GAINS (this is like the one thing that matters)
    private double main_gain, kicker_gain, main_kP, kicker_kP, kicker_kD, prevError, kicker_kF;

    private final double Talon_kF = 0.000134;

    //Constructor 
    /*Flywheel(double main_gain, double kicker_gain){
        this.main_gain = main_gain;
        this.kicker_gain = kicker_gain;
        //this._shooterright.
    }*/

    Flywheel(double main_kP, double kicker_kP, double kicker_kD){
        this.main_kP = main_kP;
        this.kicker_kP = kicker_kP;
        this.kicker_kD = kicker_kD;
        prevError = 0.0;

        kicker_kF = 1/11000;

        //Talon FX's
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = 1023.0 / 20660.0;
        config.slot0.kP = main_kP;
        config.slot0.kI = 0.0;
        config.slot0.kD = 5.0;
        config.slot0.integralZone = 0.0;
        _shooterright.configAllSettings(config);

        //Spark Max's
        //sparkmaxPID.setFF(1/11000);'
        sparkmaxPID.setFF(1/11000);
        //sparkmaxPID.setP(kicker_kP);
        sparkmaxPID.setP(0.01);
        sparkmaxPID.setI(0);
        //sparkmaxPID.setD(5.0); //might need to change
        sparkmaxPID.setD(0);
        sparkmaxPID.setIZone(0);
        sparkmaxPID.setOutputRange(-1, 1);
        //sparkmaxPID.setSmartMotionMaxVelocity(11000,0);
        //sparkmaxPID.setSmartMotionMaxAccel(200,0);
    }

    //FLYWHEEL (kicker & main) HELPER FUNCTIONS
    public double getKickerVal(double percent){
        return 12.0 * percent;
    }

    public void set(double percentOutput){
        _shooterright.set(percentOutput);
        _kickerright.setVoltage(getKickerVal(percentOutput));
    }

    //VELOCITY CONTORL (rip tbh)
    /*private double main_output = 0.0;
    private double main_tbh = 0.0;
    private double kicker_output = 0.0;
    private double kicker_tbh = 0.0;

    public void resetTBH(){
        double main_output = 0.0;
        double main_tbh = 0.0;
        double kicker_output = 0.0;
        double kicker_tbh = 0.0;
    }*/

    public void setVel(double desiredVel){ //-1 to 1
        //_shooterright.set(TalonFXControlMode.Velocity, desiredVel  * 20660.0);
        //sparkmaxPID.setReference(desiredVel * 11000, ControlType.kSmartVelocity);
        _kickerright.setVoltage(getKickerVal(getKickerPID(desiredVel * 11000)));
        SmartDashboard.putNumber("Main Velocity: ", _shooterright.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Kicker Velocity: ", _kickerEncoder.getVelocity());
    }

    public double getKickerPID(double desiredRPM){
        double currentRPM = _kickerEncoder.getVelocity();
        double error = desiredRPM - currentRPM;
        double derivative = error - prevError;
        prevError = error;
        return error * kicker_kP + derivative * kicker_kD + desiredRPM * kicker_kF;
    }

    /*public void setVelocity(double desiredVelocity, velocityControl type){
        double mainVel = _shooterright.getSelectedSensorVelocity() * (4096.0 / 60000.0);
        double kickerVel = _kickerEncoder.getVelocity();
        double desiredKickerVel = (desiredVelocity * (KICKER_FREE_SPEED / MAIN_FREE_SPEED));
        double main_error = desiredVelocity - mainVel; 
        double kicker_error = (desiredKickerVel - kickerVel);

        if(type == velocityControl.TBH){
            main_output += main_gain * main_error;
            if(Math.signum(main_error) != Math.signum(main_tbh)){
                main_output = 0.5 * (main_output + main_tbh);
                main_tbh = main_output;
            }
            _shooterright.set(main_output);

            kicker_output += kicker_gain * kicker_error;
            if(Math.signum(kicker_error) != Math.signum(kicker_tbh)){
                kicker_output = 0.5 * (kicker_output + kicker_tbh);
                kicker_tbh = kicker_output;
            }
            _kickerright.setVoltage(getKickerVal(kicker_output));

            SmartDashboard.putNumber("Main Error (TBH): ", main_error);
            SmartDashboard.putNumber("Kicker Error (TBH): ", kicker_error);
        } else {
            double _main_output = main_error * main_kP + desiredVelocity * main_kF;
            _shooterright.set(_main_output);

            double _kicker_output = kicker_error * kicker_kP + desiredKickerVel * kicker_kF;
            _kickerright.set(_kicker_output);

            SmartDashboard.putNumber("Main Error (PID): ", main_error);
            SmartDashboard.putNumber("Kicker Error (PID): ", kicker_error);
        }
    }*/
}