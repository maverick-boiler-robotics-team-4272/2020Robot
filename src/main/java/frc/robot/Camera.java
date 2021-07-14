package frc.robot;

import java.math.RoundingMode;

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

	double totalError = 0;
	double alignTurnSpeed = 0;
	double alignKP = 0.005;
	double alignKI = 0.0002;
	double maxKP = 0.2;
	double maxKI = 0.1;

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

	//Finds angle that hood should be
	

	//aims the robot based on the angle from the limelight
	public boolean updateLimelightTracking() {
		double tx = robot.tables.limelightXDegrees.getDouble(0);
		double TargetFound = robot.tables.limelightValidTarget.getDouble(0);
		robot.tables.alignTX.setDouble((tx));
		if (TargetFound != 1) {
			totalError = 0;
			alignTurnSpeed = 0;
		} else {
			totalError += tx;
			double currentTurnValue = 0;
			// if(Math.abs(alignKP * tx)>= maxKP){
			// 	currentTurnValue += ((alignKP * tx) > maxKP ? maxKP : -maxKP);
			// } else {
			// 	currentTurnValue += (alignKP * tx);
			// }
			currentTurnValue += (alignKP * tx);
			robot.tables.alignKPTErr.setDouble((alignKP * tx));
			if(Math.abs(alignKI * totalError) >= maxKI){
				currentTurnValue += ((alignKI * totalError) > maxKI ? maxKI : (maxKI*-1));
				robot.tables.alignKITErr.setDouble(((alignKI * totalError) > maxKI ? maxKI : (maxKI*-1)));
			} else {
				currentTurnValue += (alignKI * totalError);
				robot.tables.alignKITErr.setDouble((alignKI * totalError));
			}
			alignTurnSpeed = currentTurnValue;
		}

		if((Math.abs(tx) <= 1) && (TargetFound == 1)){
			return true;
		}
		return false;
	}

	public boolean updateLimelightTracking(boolean auto) {
		double tx = robot.tables.limelightXDegrees.getDouble(0);
		double TargetFound = robot.tables.limelightValidTarget.getDouble(0);
		robot.tables.alignTX.setDouble((tx));
		if (TargetFound != 1) {
			totalError = 0;
			alignTurnSpeed = 0;
		} else {
			totalError += tx;
			double currentTurnValue = 0;
			// if(Math.abs(alignKP * tx)>= maxKP){
			// 	currentTurnValue += ((alignKP * tx) > maxKP ? maxKP : -maxKP);
			// } else {
			// 	currentTurnValue += (alignKP * tx);
			// }
			currentTurnValue += (alignKP * tx);
			robot.tables.alignKPTErr.setDouble((alignKP * tx));
			if(Math.abs(alignKI * totalError) >= maxKI){
				currentTurnValue += ((alignKI * totalError) > maxKI ? maxKI : (maxKI*-1));
				robot.tables.alignKITErr.setDouble(((alignKI * totalError) > maxKI ? maxKI : (maxKI*-1)));
			} else {
				currentTurnValue += (alignKI * totalError);
				robot.tables.alignKITErr.setDouble((alignKI * totalError));
			}
			alignTurnSpeed = currentTurnValue;
		}

		if(TargetFound != 1){
			alignTurnSpeed = -0.2;
			return false;
		}

		if((Math.abs(tx) <= 1.2) && (TargetFound == 1)){
			return true;
		}
		return false;
	}
}
