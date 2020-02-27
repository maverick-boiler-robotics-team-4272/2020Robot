package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber{
    Robot robot;
    public Climber(Robot robot){
        this.robot = robot;
    }
    public void off(){
        robot.pneumatics.climberSolenoid.set(kOff);
    }
    public void up(){
        robot.pneumatics.climberSolenoid.set(kForward);
    }
    public void down(){
        robot.pneumatics.climberSolenoid.set(kReverse);
    }
}