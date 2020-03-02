package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber{
	Robot robot; //allows communication to different classes

	private boolean extended = false;

	//instanciates class
	public Climber(Robot robot){
		this.robot = robot;
	}

	public void loop(){
		//loopy type things
	}

	public void reset(){
		//things to do once
	}

	public void toggle(){
		if(extended){
			down();
			extended = false;
		}else{
			up();
			extended = true;
		}
	}

	//disables solenoid
	public void off(){
		robot.pneumatics.climberSolenoid.set(kOff);
	}

	//extends pneumatic for climber
	public void up(){
		robot.pneumatics.climberSolenoid.set(kForward);
	}

	//retracts pneumatic for climber
	public void down(){
		robot.pneumatics.climberSolenoid.set(kReverse);
	}
}
