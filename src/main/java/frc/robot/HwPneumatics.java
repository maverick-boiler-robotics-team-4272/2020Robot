package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

public class HwPneumatics{
    public Compressor c = new Compressor(0);

    public boolean enabled = c.enabled();
    public boolean pressureSwitch = c.getPressureSwitchValue();
    public double current = c.getCompressorCurrent();

    public DoubleSolenoid climberSolenoid = new DoubleSolenoid(0, 1);
    public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);
    public DoubleSolenoid CPMSolenoid = new DoubleSolenoid(4, 5);

    public HwPneumatics(){
        c.setClosedLoopControl(true);
    }

    boolean climberUp = false;
    public void climberPneumatics(boolean up){
        if(up){
            if(climberUp){
                climberSolenoid.set(DoubleSolenoid.Value.kForward);
                climberUp = false;
            }else{
                climberSolenoid.set(DoubleSolenoid.Value.kReverse);
                climberUp = true;
            }
        }
    }

    boolean CPMUp = false;
    public void CPMPneumatics(boolean up){
        if(up){
            if(CPMUp){
                CPMSolenoid.set(DoubleSolenoid.Value.kForward);
                CPMUp = false;
            }else{
                CPMSolenoid.set(DoubleSolenoid.Value.kReverse);
                CPMUp = true;
            }
        }
    }
}