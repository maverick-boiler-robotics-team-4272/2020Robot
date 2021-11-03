package frc.robot;


import java.rmi.server.RemoteObject;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.DriverStation;


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
		robot.auto.limelight = false;
		//our two joysticks
		double leftSpeed = robot.jstick.xboxDriver.getY(Hand.kLeft);
		double rightSpeed = robot.jstick.xboxDriver.getY(Hand.kRight);

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
		if (robot.jstick.xboxDriver.getBumper(Hand.kLeft)){
			robot.camera.autoAlign();
			drive(((leftSpeed+rightSpeed)/2) - robot.camera.alignTurnSpeed, ((leftSpeed+rightSpeed)/2) + robot.camera.alignTurnSpeed);
		}else if(robot.jstick.xboxDriver.getBumperReleased(Hand.kLeft)){
			robot.camera.driveVision();
			robot.camera.alignTurnSpeed = 0;
		}else{
			drive(leftSpeed, rightSpeed);
		}

		// if(!robot.jstick.leftJoystick.getTrigger()){
		// 	robot.camera.changingLed(false);
		// }

		if(robot.jstick.xboxDriver.getBumperPressed(Hand.kLeft)){
			System.out.println("Distance from goal according to LimeLight: " + robot.camera.getDistLime());
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
		if (robot.jstick.xboxDriver.getBumper(Hand.kRight)) {
            robot.hopper.shoot_balls();
            robot.shooter.startShooter();
		} else if(robot.jstick.xboxDriver.getBumperReleased(Hand.kRight)) {
            robot.hopper.stop_hopper();
            robot.shooter.stopShooter();
		}


		// this.sendIt = robot.jstick.leftJoystick.getTop();

		//sensor on/off with holding A
		// if(robot.jstick.xboxDriver.getAButtonPressed()){
		// 	robot.shooter.fixedDistanceState = true;
		// } else if (robot.jstick.xboxDriver.getAButtonReleased()){
		// 	robot.shooter.fixedDistanceState = false;
		// }

		//toggle distance sensing on/off
		if(robot.jstick.xboxDriver.getAButtonPressed()){
			if(robot.shooter.fixedDistanceState){
				robot.shooter.fixedDistanceState = false;
			}else{
				robot.shooter.fixedDistanceState = true;
			}
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
			}
			if (robot.jstick.xbox.getXButton()) {
				robot.motor.CPM.set(-0.3);
			} else {
				robot.motor.CPM.set(0);
			}
		}

		if(robot.jstick.xbox.getXButtonReleased()){
			robot.motor.CPM.set(0);
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

		if(robot.jstick.xboxDriver.getBumper(Hand.kRight)){
			robot.shooter.startShooter();
			robot.shooter.setShooterRPM();
		}else if(robot.jstick.xboxDriver.getBumperReleased(Hand.kRight)){
			robot.shooter.stopShooter();
		}
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
