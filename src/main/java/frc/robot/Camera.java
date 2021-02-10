package frc.robot;

import java.math.RoundingMode;

import edu.wpi.first.networktables.*;

/**
 * Add your docs here.
 */
public class Camera {
	Robot robot;

	// variables for limelight aiming
	public boolean m_LimelightHasValidTarget = false;
	public double m_LimelightDriveCommand = 0.0;// never move forewards or backwards
	public double m_LimelightSteerCommand = 0.0;

	public boolean is_aligning = false;
	private boolean is_driver_vision = true;
	final double STEER_K = 0.02; // max speed the robot should turn
	double steerI = 0.001;
	double accumulator = 0;
	double accumulatorMax = 2;
	double accumulatorMin = -2;
	double leftMotorSetpoint = 0;
	double rightMotorSetpoint = 0;

	public Camera(Robot robot) {
		this.robot = robot;
	}

	public void changingLed(boolean on) {
		// defalt setting for limelight network table led activation 1 being off and 3
		// being on
		int numero = 3;
		if (on) {
			// turn light on
			numero = 3;
		} else {
			// turn light off
			numero = 1;
		}
		// publish led status to limelight network table
		robot.tables.limelightLed.setNumber(3);

	}

	public void loop() {
		if (is_aligning) {
			updateLimelightTracking();
			changingLed(true);
		} else if (is_driver_vision) {
			changingLed(false);
		} else {
			System.out.println("you screwed up the robot");
		}
	}

	public void reset() {
		// things to do once
	}

	public void autoAlign() {
		is_aligning = true;
		is_driver_vision = false;
	}

	public void driveVision() {
		is_aligning = false;
		is_driver_vision = true;
	}

	public void updateLimelightTracking() {
		double UpperTolerance = -0.75;
		double BottomTolerance = -0.5;
		double tx = robot.tables.limelightXDegrees.getDouble(0);
		double error = Math.min(Math.abs(tx) - UpperTolerance, Math.abs(tx) - BottomTolerance);
		double multiplier = (Math.round(error / 6)) < 0 ? 1 : Math.round(error / 6);
		double motorSpeed = 0.4;
		double motorSpeedWMult = motorSpeed * multiplier;
		double TargetFound = robot.tables.limelightValidTarget.getDouble(0);

		if (TargetFound != 1) {
			leftMotorSetpoint = -1;
			rightMotorSetpoint = 1;
		} else {
			if (tx > UpperTolerance) {
				leftMotorSetpoint = -motorSpeedWMult;
				rightMotorSetpoint = motorSpeedWMult;
			} else if (tx < BottomTolerance) {
				leftMotorSetpoint = motorSpeedWMult;
				rightMotorSetpoint = -motorSpeedWMult;
			}
		}

		robot.motor.setLeftVelocity(leftMotorSetpoint, 0);
		robot.motor.setRightVelocity(rightMotorSetpoint, 0);

		// final double DRIVE_K = -0.26; // the feed foreward for the robot drive
		// final double DESIRED_TARGET_Y = 0; // the target y value given from limelight
		// final double MAX_DRIVE = 0.2; // max speed the robot should drive

		// //getting current network table entries

		// //found target (1 for found, 0 for not found)
		// double tv = robot.tables.limelightValidTarget.getDouble(0);

		// //x degrees from center of found target
		// double tx = robot.tables.limelightXDegrees.getDouble(0);

		// //y degrees from center of found target
		// double ty = robot.tables.limelightYDegrees.getDouble(0);
		// //end network table section

		// //test if the limelight has found a target
		// if(tv < 1.0){
		// m_LimelightHasValidTarget = false; //does not have a target
		// m_LimelightDriveCommand = 0; //dont go anywhere
		// m_LimelightSteerCommand = 0; //dont go anywhere
		// return; //stop executing the function
		// }

		// m_LimelightHasValidTarget = true; //has a valid target

		// double steer_cmd = tx * STEER_K + accumulator * steerI; // how fast it should
		// steer
		// m_LimelightSteerCommand = steer_cmd; // puts the above into an other more
		// different variable
		// accumulator += tx;
		// if(accumulator > accumulatorMax){
		// accumulator = accumulatorMax;
		// }else if(accumulator < accumulatorMin){
		// accumulator = accumulatorMin;
		// }
		// leftMotorSetpoint = m_LimelightDriveCommand - m_LimelightSteerCommand;
		// rightMotorSetpoint = m_LimelightDriveCommand + m_LimelightSteerCommand;
		// robot.motor.setLeftVelocity(leftMotorSetpoint, 0);
		// robot.motor.setRightVelocity(rightMotorSetpoint, 0);
	}
}
