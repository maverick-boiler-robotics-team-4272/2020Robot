package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.features2d.KAZE;

/**
 * Add your docs here.
 */
public class Shooter {
	Robot robot;
	public static final double SENSOR_TO_RPM = 0.07325;
	public double shooterAngleTune = 40;


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

	public double findRPM(){
		//75.7 + -0.657x + 3.63E-03x^2 + -7.56E-06x^3
		double height = 6; //6 feet
		double gravityConst = 32.17; // 32.17 ft/s
		double distance = (robot.hood.lidarLite.getDistance()/(2.53*12)+0.3); //converting LiDAR cm to ft
		double distanceInches = distance * 12;
		double C3 = -7.56E-06;
		double C2 = 3.63E-03;
		double C1 = -0.657;
		double C0 = 76.7;
		double theta = Math.toRadians((C3*Math.pow(distanceInches, 3))+(C2*Math.pow(distanceInches, 2))+(C1*distanceInches)+C0);
		double stepa = ((-0.5*(gravityConst))/(Math.pow(Math.cos(theta),2)));
		double stepb = stepa*Math.pow(distance, 2);
		double stepc = (height-(Math.tan(theta)*distance));
		Double finalSpeed = Math.pow((stepb/stepc),(0.5));
		Double finalSpeedRPM = ((finalSpeed*240)/Math.PI);
		if(!finalSpeedRPM.isNaN()){
			if(finalSpeedRPM>=6500){
				return 6500;
			}
			return finalSpeedRPM;
		}
		return 0;
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
	public void setShooterRPM(){
		System.out.println("setting shooter to run at " + findRPM() + " rpm");
		robot.motor.shooter1.set(ControlMode.Velocity, findRPM() / SENSOR_TO_RPM);
		robot.tables.shooterVelSetPoint.setNumber(findRPM());
	}

	public void setShooterRPM(double RPM){
		System.out.println("setting shooter to run at " + RPM + " rpm");
		robot.motor.shooter1.set(ControlMode.Velocity, RPM / SENSOR_TO_RPM);
		robot.tables.shooterVelSetPoint.setNumber(RPM);
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
