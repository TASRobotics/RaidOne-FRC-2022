package raidzero.robot.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.subsystems.Chassis;

public class ChassisController extends SubsystemBase {
    private Chassis chassis;

    public ChassisController(Chassis chassis) {
        this.chassis = chassis;
    }

    /**
     * Follows desired path
     * 
     * @param path desired path
     */
    public void followPath(double[] path) {
        for(int i = 0; i < path.length; i++) {
            double prevTime = Timer.getFPGATimestamp();
            chassis.moveLinearVelocity(path[i]);
            while(Timer.getFPGATimestamp() - prevTime < 0.01) {
                Timer.delay(0.001);
            }
        }
    }
}
