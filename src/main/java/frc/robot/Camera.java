/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.VictorSP;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;
/**
 * Add your docs here.
 */
public class Camera {
    public boolean m_LimelightHasValidTarget = false;
    public double m_LimelightDriveCommand = 0.0;
    public double m_LimelightSteerCommand = 0.0;
    int ledType = 1;
    int PipelineNum = 3;
    public void changingLed(){
        if(ledType == 1){
            ledType = 3;
        }
        else if(ledType == 3){
            ledType = 1;
        }
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledType);
        
    }
    public void changingPipeline(){
        if(PipelineNum == 2){
            PipelineNum = 3;
        }
        else if(PipelineNum == 3){
            PipelineNum = 2;
        }
        else if(PipelineNum == 1){
            PipelineNum = 3;
        }
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(PipelineNum);
    }
    public void updateLimelightTracking(){
        final double STEER_K = 0.005;//We'll worry about these numbers later
        final double DRIVE_K = -0.26;
        final double DESIRED_TARGET_Y = -7;
        final double MAX_DRIVE = 0.1;

        /*Didn't know what these things meant so yee.

        tv Whether the limelight has any valid targets (0 or 1)

        tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)

        ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)

        ta Target Area (0% of image to 100% of image)
        */

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        //double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        // System.out.println("tv="+tv);
        if(tv < 1.0){
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0;
            m_LimelightSteerCommand = 0.3;
            return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        // double drive_cmd = (DESIRED_TARGET_Y - ty) * DRIVE_K;
        double drive_cmd = 0;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;        
    }
}
