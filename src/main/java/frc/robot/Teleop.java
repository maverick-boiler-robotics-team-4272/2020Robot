package frc.robot;


import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class Teleop {
	private final double deadzone = 0.05;
	Robot robot;
	
	public Teleop(Robot robot){
		this.robot = robot;
	}

	boolean is_auto = false;
	double percentOutput = 0;
	public double rpm = 4000;
	boolean shootActive = false;
	private boolean climber_current_pos = false; // false is retracted true is extended
	public static boolean colorSelectionTime = false;
	boolean is_reversing = false;

	boolean current_intake_pos = false; //false is retracted

	// public static boolean reversing = false;

	public void run() {
		double leftSpeed = robot.jstick.leftJoystick.getY();
		double rightSpeed = robot.jstick.rightJoystick.getY();

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
        
        // robot.camera.loop();
        // if(robot.camera.is_aligning) {
        //     leftSpeed = robot.camera.m_LimelightDriveCommand - robot.camera.m_LimelightSteerCommand;
		// 	rightSpeed = robot.camera.m_LimelightDriveCommand + robot.camera.m_LimelightSteerCommand;
        // }
		
		
		if (robot.jstick.leftJoystick.getTrigger()) {
			robot.camera.autoAlign();
		}else if(robot.jstick.leftJoystick.getTriggerReleased()){
			robot.camera.driveVision();
		}else{
			drive(leftSpeed, rightSpeed);
		}


		if (robot.jstick.rightJoystick.getTriggerPressed()) {
            robot.hopper.shoot_balls();
            robot.shooter.startShooter();
		} else if(robot.jstick.rightJoystick.getTriggerReleased()) {
            robot.hopper.stop_hopper();
            robot.shooter.stopShooter();
		}

		if((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) > 0.15)) {
			robot.hopper.intake_balls();
			if(robot.hopper.countBalls() < 5){
				if(Math.abs(robot.jstick.xbox.getTriggerAxis(Hand.kRight)) > 0.15) {
					robot.intake.on(((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) * -1) + 0.15) * 0.75);
				}else {
					robot.intake.on((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) - 0.15) * 0.75);
				}
			}else{
				robot.intake.off();
			}
		} else {
			robot.hopper.stop_intaking();
			robot.intake.off();
		}


		if(robot.jstick.xbox.getStartButton()) {
			robot.pneumatics.compressor();
		}

		if(robot.jstick.xbox.getBackButton()) {
			robot.hopper.reverse_hopper();
			robot.intake.on(-0.5);
		}
		if(robot.jstick.xbox.getBackButtonReleased()) {
			robot.hopper.stop_hopper();
			robot.intake.off();
		}
		
		if (robot.jstick.xbox.getTriggerAxis(Hand.kRight) > 0.2) {
			
			if(robot.jstick.xbox.getAButtonPressed()) {
				robot.intake.toggle();
			}
			if (robot.jstick.xbox.getBButtonPressed()) {
				robot.pneumatics.CPMPneumatics(true);
			} else if (robot.jstick.xbox.getYButtonPressed()) {
				robot.climber.toggle();
			} else if (robot.jstick.xbox.getXButton()) {
				// robot.color.doTheColorPosition();
			}
		} else {
			// robot.color.colorRotation(robot.jstick.xbox.getAButton()); //check with operator to see what button they want assigned to this
			if(robot.jstick.xbox.getYButtonPressed()){
				// robot.color.doTheColorPosition();
				robot.motor.CPM.set(0.3);
			}else if(robot.jstick.xbox.getYButtonReleased()){
				robot.motor.CPM.set(0);
			}
		}

		if(Math.abs(robot.jstick.xbox.getY(Hand.kRight)) > 0.2){
			robot.motor.climberRight.set(robot.jstick.xbox.getY(Hand.kRight) * -1);
		}else{
			robot.motor.climberRight.set(0);
		}
		if(Math.abs(robot.jstick.xbox.getY(Hand.kLeft)) > 0.2){
			robot.motor.climberLeft.set(robot.jstick.xbox.getY(Hand.kLeft) * -1);
		}else{
			robot.motor.climberLeft.set(0);
		}

		// robot.hopper.loop(rpm);
	}

	public void drive(double leftPower, double rightPower) {
		robot.motor.left1.set(leftPower);
		robot.motor.right1.set(rightPower);
	}
}
