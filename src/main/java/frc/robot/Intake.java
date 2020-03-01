package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake{
    Robot robot;//makes sure that it can access the rest of the robot

    //instanciates the class
    public Intake(Robot robot){
        this.robot = robot;
    }

    // turns off the motors for the intake
    public void off(){
        robot.pneumatics.intakeSolenoid.set(kOff);
        robot.motor.intake.set(0);
        robot.motor.intake2.set(0);
    }

    // extends the pneumatic for the intake
    public void out(){
        robot.pneumatics.intakeSolenoid.set(kReverse);
    }

    // retracts the pneumatic for the intake
    public void in(){
        robot.pneumatics.intakeSolenoid.set(kForward);
        robot.motor.intake.set(0);
        robot.motor.intake2.set(0);
    }

    // turns on the motors for the intake
    public void on(double speed){
        robot.motor.intake.set(speed);
        robot.motor.intake2.set(-speed);
    }
}