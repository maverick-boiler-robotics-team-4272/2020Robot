package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class HwPneumatics {
	public Compressor c = new Compressor(0);//needs this so we can actually use pneumatics

	public boolean enabled = c.enabled();
	public boolean pressureSwitch = c.getPressureSwitchValue();
	public double current = c.getCompressorCurrent();
	

	public DoubleSolenoid climberSolenoid = new DoubleSolenoid(3, 4);
	public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 5);
	public DoubleSolenoid CPMSolenoid = new DoubleSolenoid(1, 6);
	public Solenoid extra = new Solenoid(0);

	public HwPneumatics(){
		c.setClosedLoopControl(true);
		climberSolenoid.set(DoubleSolenoid.Value.kForward);
		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		CPMSolenoid.set(DoubleSolenoid.Value.kForward);
		extra.set(false);
	}
	
	public void compressor(){
		if(c.getClosedLoopControl()){
			c.setClosedLoopControl(false);
		}else{
			c.setClosedLoopControl(true);
		}
	}

	

	boolean CPMUp = false;
	public void CPMPneumatics(boolean toggle){
		if(toggle){
			if(CPMUp){
				CPMSolenoid.set(DoubleSolenoid.Value.kReverse);
				CPMUp = false;
			}else{
				CPMSolenoid.set(DoubleSolenoid.Value.kForward);
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