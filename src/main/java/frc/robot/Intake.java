package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.revrobotics.ControlType;

public class Intake{
    private HwPneumatics pneumatics;
    private HwMotor motor;
    private double intake_speed = 0.4;
    public Intake(HwPneumatics pneumatics, HwMotor motor){
        this.pneumatics = pneumatics;
        this.motor = motor;
    }
    public void off(){
        pneumatics.intake.set(kOff);
        motor.intake.set(0);
    }
    public void out(){
        pneumatics.intake.set(kForward);
        motor.intake.set(intake_speed);
    }
    public void in(){
        pneumatics.intake.set(kReverse);
        motor.intake.set(0);
    }
    public void on(){
        motor.intakePID.setReference(4000, ControlType.kVelocity);
        //motor.intake.set(0.4);
    }
}