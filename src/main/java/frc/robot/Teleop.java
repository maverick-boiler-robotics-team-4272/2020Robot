/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;

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
    
    // private auto is_Auto = new auto();
    boolean is_auto = false;
    double percentOutput = 0;
    private double rpm = 3600;
    boolean shootActive = false;
    private boolean climber_current_pos = false; // false is retracted true is extended
    public static boolean colorSelectionTime = false;


    public static boolean reversing = false;

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


        if (leftSpeed > 0)
            leftSpeed *= leftSpeed;
        else
            leftSpeed *= -1 * leftSpeed;
        if (rightSpeed > 0)
            rightSpeed *= rightSpeed;
        else
            rightSpeed *= -1 * rightSpeed;

        robot.camera.updateLimelightTracking();

        if (robot.jstick.leftJoystick.getTrigger()) {
            leftSpeed = robot.camera.m_LimelightDriveCommand - robot.camera.m_LimelightSteerCommand;
            rightSpeed = robot.camera.m_LimelightDriveCommand + robot.camera.m_LimelightSteerCommand;
            drive(leftSpeed, rightSpeed);
        } else {
            drive(leftSpeed, rightSpeed);
        }

        if (Math.abs(robot.jstick.xbox.getY(Hand.kLeft)) > 0.5) {
            rpm -= 10 * robot.jstick.xbox.getY(Hand.kLeft);
        }
        robot.shooter.sendNumbers();

        if (shootActive) {
            robot.shooter.setShooterRPM(rpm);
        } else {
            robot.shooter.shooterDisable();
        }

        if (robot.jstick.xbox.getBumperPressed(Hand.kLeft)) {
            robot.camera.changingLed();
        }
        if (robot.jstick.xbox.getBumperPressed(Hand.kRight)) {
            robot.camera.changingPipeline();
        }

        if (robot.jstick.xbox.getYButtonPressed()) {
            if (climber_current_pos) {
                climber_current_pos = false;
                robot.climber.up();
            } else {
                climber_current_pos = true;
                robot.climber.down();
            }
        }

        // manually run the intake
        robot.hopper.readArduino(); // update sensor values
        if (robot.jstick.rightJoystick.getTriggerPressed()) {
            robot.hopper.shoot_balls();
            shootActive = true;
        } else if(robot.jstick.rightJoystick.getTriggerReleased()){
            robot.hopper.reverse_balls();
            shootActive = false;
        }

        if(robot.jstick.xbox.getTriggerAxis(Hand.kLeft) > 0.3){
            robot.hopper.movement(false);
            robot.intake.out();
        } else {
            robot.hopper.disable();
            robot.intake.in();
        }
        
        robot.motor.ball1.setBoolean(true);
        robot.motor.ball2.setBoolean(true);
        robot.motor.ball3.setBoolean(true);
        robot.motor.ball4.setBoolean(true);
        robot.motor.ball5.setBoolean(true);

        if (robot.jstick.xbox.getTriggerAxis(Hand.kRight) > 0.2) {
            if(robot.jstick.xbox.getAButton()){
                robot.motor.CPM.set(0.5);
            }
            if (robot.jstick.xbox.getBButton()) {
                // robot.color.colorSelection("Red");
                robot.pneumatics.CPMPneumatics(true);
            } else if (robot.jstick.xbox.getYButton()) {
                // robot.color.colorSelection("Yellow");\
                robot.pneumatics.climberPneumatics(true);
            } else if (robot.jstick.xbox.getXButton()) {
                robot.color.doTheColorPosition();
            }
        } else {
            robot.color.colorRotation(robot.jstick.xbox.getAButton());
        }
        robot.hopper.loop(rpm);
    }

    public void drive(double leftPower, double rightPower) {
        robot.motor.left1.set(leftPower);
        robot.motor.right1.set(rightPower);
    }
}