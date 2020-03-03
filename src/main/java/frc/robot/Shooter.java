package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class Shooter {
	Robot robot;
	public static final double SENSOR_TO_RPM = 0.07325;

	private double rpm = robot.teleop.rpm;

	private boolean is_shooting = false;

	public Shooter(Robot robot) {
		this.robot = robot;
	}

	public void loop(){
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

	private void setShooterRPM(){
		robot.motor.shooter1.set(ControlMode.Velocity, rpm / SENSOR_TO_RPM);
		robot.tables.shooterVelSetPoint.setNumber(rpm);
	}

	private void sendNumbers(){
		robot.tables.shooterVel.setNumber(SENSOR_TO_RPM * robot.motor.shooter1.getSelectedSensorVelocity());
		robot.tables.shooterOutput.setNumber(robot.motor.shooter1.getMotorOutputPercent());
	}

	private void shooterDisable(){
		robot.motor.shooter1.set(ControlMode.PercentOutput, 0);
	}

}
