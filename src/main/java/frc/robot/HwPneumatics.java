package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class HwPneumatics {
    public Compressor c = new Compressor(0);

    public boolean enabled = c.enabled();
    public boolean pressureSwitch = c.getPressureSwitchValue();
    public double current = c.getCompressorCurrent();

    public DoubleSolenoid climberSolenoid = new DoubleSolenoid(1, 6);
    public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 5);
    public DoubleSolenoid CPMSolenoid = new DoubleSolenoid(3, 4);
    public Solenoid extra = new Solenoid(0);
    public HwPneumatics(){
        c.setClosedLoopControl(true);
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        CPMSolenoid.set(DoubleSolenoid.Value.kReverse);
        extra.set(false);
    }

    boolean climberUp = false;
    public void climberPneumatics(boolean up){
        if(up){
            if(climberUp){
                climberSolenoid.set(DoubleSolenoid.Value.kForward);
                extra.set(false);
                climberUp = false;
            }else{
                climberSolenoid.set(DoubleSolenoid.Value.kReverse);
                extra.set(true);
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

    boolean intakeOut = false;
    public void intakePneumatics(boolean toggle) {
        if(toggle) {
            if(intakeOut) {
                intakeSolenoid.set(DoubleSolenoid.Value.kForward);
                intakeOut = false;
            } else {
                intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                intakeOut = true;
            }
        }
    }
}