/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class Shooter {
    Robot robot;
    public static final double SENSOR_TO_RPM = 0.07325;

    public Shooter(Robot robot) {
        this.robot = robot;
    }

    public void setShooterRPM(double rpm){
        robot.motor.shooter1.set(ControlMode.Velocity, rpm / SENSOR_TO_RPM);
        // motor.shooter1.set(ControlMode.PercentOutput, -0.2);
        robot.motor.shooterVelSetPoint.setNumber(rpm);
    }

    //Temporary hopper code??
    public void sendTheHopper(double speed){
        robot.motor.hopper.set(-speed);
        robot.motor.hopper_infeed.set(-speed);
    }

    public void miniShootBOI(double speed){
        robot.motor.miniShooter.set(-speed);
    }

    public void sendNumbers(){
        robot.motor.shooterVel.setNumber(SENSOR_TO_RPM * robot.motor.shooter1.getSelectedSensorVelocity());
        // System.out.println("motor power: " + motor.shooter1.getMotorOutputPercent());
        robot.motor.shooterOutput.setNumber(robot.motor.shooter1.getMotorOutputPercent());
    }

    public void shooterDisable(){
        robot.motor.shooter1.set(ControlMode.PercentOutput, 0);
    }

}
