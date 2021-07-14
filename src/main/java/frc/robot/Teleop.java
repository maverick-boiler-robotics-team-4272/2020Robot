package frc.robot;


import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class Teleop {
	private final double deadzone = 0.1;
	Robot robot;
	
	public Teleop(Robot robot){
		this.robot = robot;
	}

	boolean is_auto = false;
	double percentOutput = 0;
	boolean shootActive = false;
	private boolean climber_current_pos = false; // false is retracted true is extended
	public static boolean colorSelectionTime = false;
	boolean is_reversing = false;

	boolean current_intake_pos = false; //false is retracted
	double cpmEncoderStart = 0;
	double cpmEncoderEnd = 150;
	boolean colorRotation = false;

	double hoodAngle = 0;

	public boolean sendIt;

	// boolean wasZeroRight = true;
	// double rightAcceleration = 0;
	// boolean wasZeroLeft = true;
	// double leftAcceleration = 0;

	// public static boolean reversing = false;

	public void run() {
		//our two joysticks
		double leftSpeed = robot.jstick.leftJoystick.getY();
		double rightSpeed = robot.jstick.rightJoystick.getY();

		//make sure that it does not randomly move on its own we have a deadzone
		if (Math.abs(leftSpeed) < deadzone) {
			leftSpeed = 0;
		} else {
			if(leftSpeed > 0) {
				leftSpeed -= deadzone;
				leftSpeed /= (1 - deadzone);
			} else {
				leftSpeed += deadzone;
				leftSpeed /= (1 - deadzone);
			}
		}

		if (Math.abs(rightSpeed) < deadzone) {
			rightSpeed = 0;
		} else {
			if(rightSpeed > 0) {
				rightSpeed -= deadzone;
				rightSpeed /= (1 - deadzone);
			} else {
				rightSpeed += deadzone;
				rightSpeed /= (1 - deadzone);
			}
		}


		// use so we can still get low speeds, and have control of higher speeds as well
		if (leftSpeed > 0) {
			leftSpeed *= leftSpeed;
		}else{
			leftSpeed *= -1 * leftSpeed;
		}

		if (rightSpeed > 0) {
			rightSpeed *= rightSpeed;
		}else{
			rightSpeed *= -1 * rightSpeed;
		}
		
		
		//on our left joystick the front trigger button, that activates our auto aligning code for the limelight
		if (robot.jstick.leftJoystick.getTrigger()) {
			robot.camera.autoAlign();
			drive(((leftSpeed+rightSpeed)/2) - robot.camera.alignTurnSpeed, ((leftSpeed+rightSpeed)/2) + robot.camera.alignTurnSpeed);
		}else if(robot.jstick.leftJoystick.getTriggerReleased()){
			robot.camera.driveVision();
			robot.camera.alignTurnSpeed = 0;
		}else{
			drive(leftSpeed, rightSpeed);
		}

		if(robot.jstick.leftJoystick.getTriggerPressed()){
			robot.camera.totalError = 0;
		}

		//the slow version of the shooter
		// if(robot.jstick.rightJoystick.getTopPressed()){
		// 	rpm = rpmLow;

		// 	robot.hopper.shoot_balls();
		// 	robot.shooter.startShooter();
		// }else if(robot.jstick.rightJoystick.getTopReleased()){
		// 	robot.hopper.stop_hopper();
		// 	robot.shooter.stopShooter();
		// }


		//the fast version of the shooter
		if (robot.jstick.rightJoystick.getTriggerPressed()) {
            robot.hopper.shoot_balls();
            robot.shooter.startShooter();
		} else if(robot.jstick.rightJoystick.getTriggerReleased()) {
            robot.hopper.stop_hopper();
            robot.shooter.stopShooter();
		}


		this.sendIt = robot.jstick.leftJoystick.getTop();


		//to allow control from the joystick (not working right now)
		if(robot.jstick.rightJoystick.getRawButtonPressed(4)) {
			robot.hopper.intake_balls();
			if(robot.hopper.countBalls() < 5){
				robot.intake.on(1);
			}
		} else if(robot.jstick.rightJoystick.getRawButtonReleased(4)){
			robot.hopper.stop_intaking();
			robot.intake.off();
		}
		if(robot.jstick.rightJoystick.getRawButtonReleased(3)){
			robot.intake.toggle();
		}
		

		//the code to allow variable speed the intake
		if((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) > 0.15)) {
			robot.hopper.intake_balls();
			if(robot.hopper.countBalls() < 5){
				if(Math.abs(robot.jstick.xbox.getTriggerAxis(Hand.kRight)) > 0.15) {
					//reverses the intake is the left trigger is pressed in tandom with the right one
					robot.intake.on((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) + 0.15) * 0.75);
				}else {
					robot.intake.on(((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) * -1) - 0.15) * 0.75);
				}
			}else{
				robot.intake.off();
			}
		} else {
			robot.hopper.stop_intaking();
			robot.intake.off();
		}


		// runs the compressor with the start button
		if(robot.jstick.xbox.getStartButton()) {
			robot.pneumatics.compressor();
		}

		// reverses the hopper in the event that a ball gets stuck
		if(robot.jstick.xbox.getBackButton()) {
			robot.hopper.reverse_hopper();
			robot.intake.on(-0.5);
		}
		if(robot.jstick.xbox.getBackButtonReleased()) {
			robot.hopper.stop_hopper();
			robot.intake.off();
		}
		
		//the right trigger is acting as a button for us
		if (robot.jstick.xbox.getTriggerAxis(Hand.kRight) > 0.2) {
			
			if(robot.jstick.xbox.getAButtonPressed()) {
				robot.intake.toggle();
			}
			if (robot.jstick.xbox.getBButtonPressed()) {
				robot.pneumatics.CPMPneumatics(true);
			} else if (robot.jstick.xbox.getYButtonPressed()) {
				robot.climber.toggle();
			} else if (robot.jstick.xbox.getXButtonPressed()) {
				colorRotationInit();
				colorRotation = true;
			}
		} else {
			// robot.color.colorRotation(robot.jstick.xbox.getAButton()); //check with operator to see what button they want assigned to this
			if(robot.jstick.xbox.getYButtonPressed()){
				// robot.color.doTheColorPosition();
				robot.motor.CPM.set(0.7);
			}else if(robot.jstick.xbox.getYButtonReleased()){
				robot.motor.CPM.set(0);
			}
		}

		if(colorRotation){
			colorRotation();
		}

		// individually control the the climber hooks t allow for leveling
		if(Math.abs(robot.jstick.xbox.getY(Hand.kRight)) > 0.2){
			robot.motor.climberRight.set(robot.jstick.xbox.getY(Hand.kRight) * -1);
			System.out.println(robot.motor.climberRight.getEncoder().getPosition());
		}else{
			robot.motor.climberRight.set(0);
		}
		if(Math.abs(robot.jstick.xbox.getY(Hand.kLeft)) > 0.2){
			robot.motor.climberLeft.set(robot.jstick.xbox.getY(Hand.kLeft) * -1);
			System.out.println(robot.motor.climberLeft.getEncoder().getPosition());
		}else{
			robot.motor.climberLeft.set(0);
		}

		//allow for manual release of the soft limit due to it being set at the very beginning
		if(robot.jstick.leftJoystick.getRawButtonPressed(16)) {
			robot.motor.climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
			robot.motor.climberRight.enableSoftLimit(SoftLimitDirection.kReverse, false);
			System.out.println("disabled climber limits!!!");
		} else if(robot.jstick.leftJoystick.getRawButtonReleased(16)) {
			robot.motor.climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
			robot.motor.climberRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
			System.out.println("re-enabled climber limits");
		}

		if(robot.jstick.leftJoystick.getRawAxis(3) > -0.9){
			// given (-0.9, 1)
			// want (0.5, 13)
			// y = (x + 0.9) * ((13 - 0.5) / (1 - -0.9)) + 0.5
			robot.motor.hoodPID.setReference((robot.jstick.leftJoystick.getRawAxis(3) + 0.9) * (12.5/1.9) + 0.5, ControlType.kPosition);
		}

		if(robot.jstick.leftJoystick.getRawButton(3)){
			System.out.println("Current Hood Position = " + robot.motor.shooterHood.getEncoder().getPosition());
		}

		if(robot.jstick.leftJoystick.getRawButtonPressed(7)){
			//1st range shot(Distance = 62.69)
			robot.motor.hoodPID.setReference(4.0, ControlType.kPosition);
			System.out.println("short range");
		}
		if(robot.jstick.leftJoystick.getRawButtonPressed(6)){
			//2nd range shot(Distance = 114.06)
			robot.motor.hoodPID.setReference(5.55, ControlType.kPosition);
			System.out.println("2nd range");
		}
		if(robot.jstick.leftJoystick.getRawButtonPressed(5)){
			//3rd range shot(Distance = 192.68)
			robot.motor.hoodPID.setReference(5.38, ControlType.kPosition);
			System.out.println("3rd range");
		}
		if(robot.jstick.leftJoystick.getRawButtonPressed(8)){
			//4th range shot(Distance = 252.04)
			robot.motor.hoodPID.setReference(4.88/*(maybe 5.83)*/, ControlType.kPosition);
			System.out.println("long range");
		}
		if(robot.jstick.rightJoystick.getRawButton(7)){
			robot.shooter.startShooter();
			robot.shooter.setShooterRPM();
		}else if(robot.jstick.rightJoystick.getRawButtonReleased(7)){
			robot.shooter.stopShooter();
		}

		// robot.tables.shooterDistance.setDouble(robot.hood.limeLightDegreesToDistance(robot.tables.limelightYDegrees.getDouble(0)));
		// robot.tables.shooterDistance.setDouble(1.0);

		//Setting Hood Angle with math
		if(robot.jstick.leftJoystick.getRawButtonPressed(2)){
			robot.hood.getHoodAngle();
			hoodAngle = robot.hood.getHoodAngle();
		}
		if(robot.jstick.leftJoystick.getRawButton(2)){
			robot.hood.goToAngle(hoodAngle);
		}else if(robot.jstick.leftJoystick.getRawButtonReleased(2)){
			// robot.motor.shooterHood.set(0);
			// hoodAngle = 0;
		}

		if(robot.jstick.rightJoystick.getRawButton(2)){
			robot.motor.resetHoodPosition();
		}
		// robot.hood.getLidarDistance();
		// System.out.println("" + robot.hood.lidarLite.getDistance());
	}



	public void drive(double leftPower, double rightPower) {
		robot.motor.left1.set(leftPower);
		robot.motor.right1.set(rightPower);
	}

	public void colorRotation(){
		if(robot.motor.CPM.getEncoder().getPosition() < cpmEncoderStart + cpmEncoderEnd){
			robot.motor.CPM.set(0.7);
		}else{
			robot.motor.CPM.set(0);
			colorRotation = false;
		}
	}

	public void colorRotationInit(){
		cpmEncoderStart = robot.motor.CPM.getEncoder().getPosition();
	}
}
