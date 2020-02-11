package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class HwPneumatics{
    public Compressor c = new Compressor(0);

    public boolean enabled = c.enabled();
    public boolean pressureSwitch = c.getPressureSwitchValue();
    public double current = c.getCompressorCurrent();

    public DoubleSolenoid climber = new DoubleSolenoid(0, 1);
    public DoubleSolenoid intake = new DoubleSolenoid(2, 3);
    public DoubleSolenoid unclaimed3 = new DoubleSolenoid(4, 5);
    public Solenoid unclaimed4 = new Solenoid(6);

    public HwPneumatics(){
        c.setClosedLoopControl(true);
    }
}