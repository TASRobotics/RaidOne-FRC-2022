package raidzero.robot.wrappers;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import raidzero.robot.Constants;

public class InactiveDoubleSolenoid extends DoubleSolenoid {
    
    protected boolean active = true;

    public InactiveDoubleSolenoid(final int forwardChannel, final int reverseChannel) {
        super(Constants.DriveConstants.PNEUMATICS_MODULE_TYPE, forwardChannel, reverseChannel);
    }
 
    public void setActive(boolean state) {
        active = state;
    }
  
    @Override
    public void set(Value value) {
        if (!active) {
            return;
        }
        super.set(value);
    }
}