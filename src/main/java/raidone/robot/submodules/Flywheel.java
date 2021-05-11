package raidone.robot.submodules;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Percentage;

//Talon FX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
//Sparkmax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
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

    //GAINS (this is like the one thing that matters)
    private double main_kP, kicker_kP, kicker_kD, prevError, kicker_kF;

    //Constructor 
    public Flywheel(double main_kP, double kicker_kP, double kicker_kD){
        this.main_kP = main_kP;
        this.kicker_kP = kicker_kP;
        this.kicker_kD = kicker_kD;
        this.prevError = 0.0;

        this.kicker_kF = 1.0/11000.0;

        //Talon FX's
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = 1023.0 / 20660.0;
        config.slot0.kP = main_kP;
        config.slot0.kI = 0.0;
        config.slot0.kD = 5.0;
        config.slot0.integralZone = 0.0;
        _shooterright.configAllSettings(config);
        _shooterleft.setInverted(InvertType.InvertMotorOutput);
        _shooterleft.follow(_shooterright, FollowerType.PercentOutput);
    }

    //FLYWHEEL (kicker & main) HELPER FUNCTIONS
    public double getKickerVal(double percent){
        return 12.0 * percent;
    }

    public double limit_absOne(double val){
        return Math.abs(val) > 1 ? 1 * Math.signum(val) : val;
    }

    public void set(double percentOutput){
        _shooterright.set(percentOutput);
        _kickerright.setVoltage(getKickerVal(percentOutput));
    }

    public void setVel(double desiredVel){ //-1 to 1
        _shooterright.set(TalonFXControlMode.Velocity, desiredVel  * 20660.0);
        _kickerright.setVoltage(getKickerVal(limit_absOne(getKickerPID(desiredVel * 11000))));
        SmartDashboard.putNumber("Main Velocity: ", _shooterright.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Kicker Velocity: ", _kickerEncoder.getVelocity());
    }

    public double getKickerPID(double desiredRPM){
        double currentRPM = _kickerEncoder.getVelocity();
        double error = desiredRPM - currentRPM;
        double derivative = error - prevError;
        prevError = error;
        SmartDashboard.putNumber("Kicker Error", error);
        return error * kicker_kP + derivative * kicker_kD + desiredRPM * kicker_kF;
    }
}