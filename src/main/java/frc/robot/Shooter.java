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
    private HwMotor motor;
    public static final double SENSOR_TO_RPM = 0.07325;

    public Shooter(HwMotor motor) {
        this.motor = motor;
    }

    public void setShooterRPM(double rpm){
        motor.shooter1.set(ControlMode.Velocity, rpm / SENSOR_TO_RPM);
        // motor.shooter1.set(ControlMode.PercentOutput, -0.2);
        motor.shooterVelSetPoint.setNumber(rpm);
    }

    //Temporary hopper code??
    public void sendTheHopper(double speed){
        motor.hopper.set(-speed);
        motor.hopper_infeed.set(-speed);
    }

    public void miniShootBOI(double speed){
        motor.miniShooter.set(-speed);
    }

    public void sendNumbers(){
        motor.shooterVel.setNumber(SENSOR_TO_RPM * motor.shooter1.getSelectedSensorVelocity());
        // System.out.println("motor power: " + motor.shooter1.getMotorOutputPercent());
        motor.shooterOutput.setNumber(motor.shooter1.getMotorOutputPercent());
    }

    public void shooterDisable(){
        motor.shooter1.set(ControlMode.PercentOutput, 0);
    }

}
