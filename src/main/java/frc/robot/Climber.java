package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber{
    private HwPneumatics pneumatics;
    public Climber(HwPneumatics pneumatics){
        this.pneumatics = pneumatics;
    }
    public void off(){
        pneumatics.climber.set(kOff);
    }
    public void up(){
        pneumatics.climber.set(kForward);
    }
    public void down(){
        pneumatics.climber.set(kReverse);
    }
}