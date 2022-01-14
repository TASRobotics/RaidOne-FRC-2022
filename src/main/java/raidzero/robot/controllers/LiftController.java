package raidzero.robot.controllers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.subsystems.Lift;

public class LiftController extends SubsystemBase {
    private Lift lift;

    public LiftController(Lift lift) {
        this.lift = lift;
    }

    public void moveTo(double pos) {
        // some sort of closed loop positional controller
    }
}
