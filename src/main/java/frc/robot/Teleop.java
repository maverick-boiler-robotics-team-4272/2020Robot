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

    private HwJoystick jstick = new HwJoystick();
    private Camera camera = new Camera();
    public HwMotor motor = new HwMotor();
    private Hopper hopper = new Hopper(motor);
    private Shooter shooter = new Shooter(motor);
    private HwPneumatics pneumatics = new HwPneumatics();
    private Climber climber = new Climber(pneumatics);
    private Intake intake = new Intake(pneumatics, motor);
    private ColorThing color = new ColorThing(motor);
    // private auto is_Auto = new auto();
    boolean is_auto = false;
    double percentOutput = 0;
    private double rpm = 3600;
    boolean shootActive = false;
    private boolean climber_current_pos = false; // false is retracted true is extended
    public static boolean colorSelectionTime = false;


    public static boolean reversing = false;

    public void run() {
        double leftSpeed = jstick.leftJoystick.getY();
        double rightSpeed = jstick.rightJoystick.getY();
        // double WheelSpeed = jstick.xbox.getTriggerAxis(Hand.kRight);

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

        /*
         * if(Math.abs(WheelSpeed) < 0.1){ WheelSpeed = 0; }
         */

        if (leftSpeed > 0)
            leftSpeed *= leftSpeed;
        else
            leftSpeed *= -1 * leftSpeed;
        if (rightSpeed > 0)
            rightSpeed *= rightSpeed;
        else
            rightSpeed *= -1 * rightSpeed;
        // if(jstick.xbox.getYButtonPressed()){
        // is_auto = true;
        // }
        // if(jstick.xbox.getAButtonPressed()){
        // is_auto = false;
        // }
        // if(is_auto){
        // Auto.drive(0.5);
        // }

        camera.updateLimelightTracking();

        if (jstick.leftJoystick.getTrigger()) {
            leftSpeed = camera.m_LimelightDriveCommand - camera.m_LimelightSteerCommand;
            rightSpeed = camera.m_LimelightDriveCommand + camera.m_LimelightSteerCommand;
            drive(leftSpeed, rightSpeed);
        } else {
            drive(leftSpeed, rightSpeed);
        }

        if (Math.abs(jstick.xbox.getY(Hand.kLeft)) > 0.5) {
            rpm -= 10 * jstick.xbox.getY(Hand.kLeft);
        }
        // shootActive = jstick.rightJoystick.getTrigger();
        if (jstick.xbox.getStartButtonPressed()) {
            shootActive = true;
        } else if (jstick.xbox.getBackButtonPressed()) {
            shootActive = false;
        }
        shooter.sendNumbers();

        if (shootActive) {
            shooter.setShooterRPM(rpm);
        } else {
            shooter.shooterDisable();
        }

        if (jstick.xbox.getBumperPressed(Hand.kLeft)) {
            camera.changingLed();
        }
        if (jstick.xbox.getBumperPressed(Hand.kRight)) {
            camera.changingPipeline();
        }

        if (jstick.xbox.getYButtonPressed()) {
            if (climber_current_pos) {
                climber_current_pos = false;
                climber.up();
            } else {
                climber_current_pos = true;
                climber.down();
            }
        }

        // manually run the intake
        hopper.readArduino(); // update sensor values
        if (jstick.rightJoystick.getTrigger()) {
            hopper.movement(true, rpm);
            intake.off();
        } else if(jstick.rightJoystick.getTriggerReleased()){
            reversing = true;
        } else {
            if(jstick.xbox.getTriggerAxis(Hand.kLeft) > 0.1)
                motor.miniShooter.set(-1 * jstick.xbox.getTriggerAxis(Hand.kLeft));
            else
                hopper.disable();
            motor.intake.set(0);
        }
        motor.intakeVel.setNumber(motor.intakeEncoder.getVelocity());
        motor.intakeTemp.setNumber(motor.intake.getMotorTemperature());

        // if(jstick.xbox.getTriggerAxis(Hand.kRight)>0.3){
        // control.hopper.movement(true);
        // }
        if (jstick.xbox.getTriggerAxis(Hand.kRight) > 0) {
            if (jstick.xbox.getAButton()) {
                color.colorSelection("Green");
            } else if (jstick.xbox.getBButton()) {
                color.colorSelection("Red");
            } else if (jstick.xbox.getYButton()) {
                color.colorSelection("Yellow");
            } else if (jstick.xbox.getXButton()) {
                color.colorSelection("Blue");
            }
        } else {
            color.colorRotation(jstick.xbox.getAButton());
        }

    }

    public void drive(double leftPower, double rightPower) {
        rightPower = -1 * rightPower;
        motor.left1.set(leftPower);
        motor.left2.set(leftPower);
        motor.right1.set(rightPower);
        motor.right2.set(rightPower);
    }
}