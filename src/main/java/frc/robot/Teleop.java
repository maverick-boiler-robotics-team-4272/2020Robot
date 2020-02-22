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
    boolean is_auto = false;
    double percentOutput = 0;
    private double rpm = 3600;
    boolean shootActive = false;
    private boolean climber_current_pos = false; // false is retracted true is extended
    public static boolean colorSelectionTime = false;
    boolean is_reversing = false;

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


        if (leftSpeed > 0){
            leftSpeed *= leftSpeed;
        }else{
            leftSpeed *= -1 * leftSpeed;
        }

        if (rightSpeed > 0){
            rightSpeed *= rightSpeed;
        }else{
            rightSpeed *= -1 * rightSpeed;
        }
        robot.camera.updateLimelightTracking();

        if (robot.jstick.leftJoystick.getTrigger()) {
            leftSpeed = robot.camera.m_LimelightDriveCommand - robot.camera.m_LimelightSteerCommand;
            rightSpeed = robot.camera.m_LimelightDriveCommand + robot.camera.m_LimelightSteerCommand;
            drive(leftSpeed, rightSpeed);
        } else {
            drive(leftSpeed, rightSpeed);
        }

        // if (Math.abs(robot.jstick.xbox.getY(Hand.kLeft)) > 0.5) {
        //     rpm -= 10 * robot.jstick.xbox.getY(Hand.kLeft);
        // }

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

        // if (robot.jstick.xbox.getYButtonPressed()) {
        //     if (climber_current_pos) {
        //         climber_current_pos = false;
        //         robot.climber.up();
        //     } else {
        //         climber_current_pos = true;
        //         robot.climber.down();
        //     }
        // }

        // manually run the intake
        robot.hopper.readArduino(); // update sensor values
        if (robot.jstick.rightJoystick.getTriggerPressed()) {
            robot.hopper.shoot_balls();
            shootActive = true;
            //is_reversing = false;
        } else if(robot.jstick.rightJoystick.getTriggerReleased()&&!is_reversing){
            //is_reversing = true;
            robot.hopper.reverse_balls();
            shootActive = false;
        }

        if(robot.jstick.xbox.getTriggerAxis(Hand.kLeft) > 0.15){
            robot.hopper.movement(false, false);
            if(Math.abs(robot.jstick.xbox.getTriggerAxis(Hand.kRight)) > 0.15){
                robot.intake.out((robot.jstick.xbox.getTriggerAxis(Hand.kLeft) * -1) + 0.15);
            }else{
                robot.intake.out(robot.jstick.xbox.getTriggerAxis(Hand.kLeft) - 0.15);
            }
        } else {
            robot.hopper.disable();
            robot.intake.in();
        }

        if(robot.jstick.xbox.getStartButton()){
            robot.motor.intake.set(0.4);
            robot.motor.hopper_infeed.set(0.4);
            robot.motor.hopper.set(0.4);
        }else if(robot.jstick.xbox.getStartButtonReleased()){
            robot.motor.intake.set(0);
            robot.motor.hopper_infeed.set(0);
            robot.motor.hopper.set(0);
        }
        
        if (robot.jstick.xbox.getTriggerAxis(Hand.kRight) > 0.2) {
            
            if(robot.jstick.xbox.getAButtonPressed()) {
                robot.pneumatics.intakePneumatics(true);
                // robot.motor.CPM.set(0.5);
            }
            if (robot.jstick.xbox.getBButtonPressed()) {
                robot.pneumatics.CPMPneumatics(true);
            } else if (robot.jstick.xbox.getYButtonPressed()) {
                robot.pneumatics.climberPneumatics(true);
            } else if (robot.jstick.xbox.getXButton()) {
                robot.color.doTheColorPosition();
            }
        } else {
            robot.color.colorRotation(robot.jstick.xbox.getAButton());
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

        robot.hopper.loop(rpm);
        robot.shooter.sendNumbers();
        robot.motor.ball1.setBoolean(robot.hopper.intake_to_hopper_sensor);
        robot.motor.ball2.setBoolean(robot.hopper.prev_intake_to_hopper_sensor);
        robot.motor.ball3.setBoolean(robot.hopper.hopper_ball_a);
        robot.motor.ball4.setBoolean(robot.hopper.hopper_ball_b);
        robot.motor.ball5.setBoolean(robot.hopper.hopper_ball_c);
    }

    public void drive(double leftPower, double rightPower) {
        robot.motor.left1.set(leftPower);
        robot.motor.right1.set(rightPower);
    }
}