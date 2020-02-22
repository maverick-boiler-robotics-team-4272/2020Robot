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
        // pneumatics.intakeSolenoid.set(kOff);
        motor.intake.set(0);
    }
    public void out(double speed){
        // pneumatics.intakeSolenoid.set(kForward);
        motor.intake.set(speed);
        motor.intake2.set(-speed);
    }
    public void in(){
        // pneumatics.intakeSolenoid.set(kReverse);
        motor.intake.set(0);
        motor.intake2.set(0);
    }
    public void on(){
        motor.intakePID.setReference(4000, ControlType.kVelocity);
        //motor.intake.set(0.4);
    }
}