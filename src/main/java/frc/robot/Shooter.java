package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class Shooter {
	Robot robot;
	public static final double SENSOR_TO_RPM = 0.07325;

	private double rpm = 0;

	private boolean is_shooting = false;

	public Shooter(Robot robot) {
		this.robot = robot;
	}

	public void loop(){
		rpm = robot.teleop.rpm;
		sendNumbers();
		if(is_shooting){
			setShooterRPM();
		} else {
			shooterDisable();
		}
	}

	public void reset(){
		//things to do once
	}

	public void startShooter(){
		is_shooting = true;
	}

	public void stopShooter(){
		is_shooting = false;
	}

	//running the shooter at the specified speed using the PID controller
	private void setShooterRPM(){
		System.out.println("setting shooter to run at " + rpm + " rpm");
		robot.motor.shooter1.set(ControlMode.Velocity, rpm / SENSOR_TO_RPM);
		robot.tables.shooterVelSetPoint.setNumber(rpm);
	}

	//put the current numbers into network tables
	private void sendNumbers(){
		robot.tables.shooterVel.setNumber(SENSOR_TO_RPM * robot.motor.shooter1.getSelectedSensorVelocity());
		robot.tables.shooterOutput.setNumber(robot.motor.shooter1.getMotorOutputPercent());
	}

	// turns the shooter off
	private void shooterDisable(){
		robot.motor.shooter1.set(ControlMode.PercentOutput, 0);
	}

	

}
