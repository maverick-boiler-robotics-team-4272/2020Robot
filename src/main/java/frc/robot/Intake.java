package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake{
    Robot robot;
    public Intake(Robot robot){
        this.robot = robot;
    }
    public void off(){
        robot.pneumatics.intakeSolenoid.set(kOff);
        robot.motor.intake.set(0);
    }
    public void out(double speed){
        robot.pneumatics.intakeSolenoid.set(kReverse);
        robot.motor.intake.set(speed);
        robot.motor.intake2.set(-speed);
    }
    public void in(){
        robot.pneumatics.intakeSolenoid.set(kForward);
        robot.motor.intake.set(0);
        robot.motor.intake2.set(0);
    }
    public void on(){
        robot.motor.intakePID.setReference(4000, ControlType.kVelocity);
        //robot.motor.intake.set(0.4);
    }
}