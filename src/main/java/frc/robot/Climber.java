package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber{
    private HwPneumatics pneumatics;
    public Climber(HwPneumatics pneumatics){
        this.pneumatics = pneumatics;
    }
    public void off(){
        pneumatics.climberSolenoid.set(kOff);
    }
    public void up(){
        pneumatics.climberSolenoid.set(kForward);
    }
    public void down(){
        pneumatics.climberSolenoid.set(kReverse);
    }
}