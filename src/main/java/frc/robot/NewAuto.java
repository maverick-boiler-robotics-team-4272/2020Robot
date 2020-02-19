/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class NewAuto {
    HwMotor motor;
    RamseteController ramsete = new RamseteController();
    DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));
    DifferentialDriveKinematicsConstraint driveContraint = new DifferentialDriveKinematicsConstraint(driveKinematics, 10);
    public DifferentialDriveOdometry driveOdometry;
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(5));
    public double autoStartTime;
    public Trajectory trajectory;
    double RPM_TO_MPS = (9/84) * Units.inchesToMeters(6) * Math.PI;
    DifferentialDriveWheelSpeeds prevSpeeds;
    double prevTime;
    double curTime;
    private Supplier<Pose2d> pose;


    public NewAuto(HwMotor motor){
        this.motor = motor;
    }

    public void trajectoryConfig() {

    }

    public void generateTrajectory() {
        driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1 * motor.ahrs.getFusedHeading()));
        config.addConstraint(driveContraint);
        var sideStart = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(-180));
        var crossScale = new Pose2d(Units.feetToMeters(1), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
        var interiorWaypoints = new ArrayList<Translation2d>();
        // interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(0)));
        // interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.75), Units.feetToMeters(0)));
        trajectory = TrajectoryGenerator.generateTrajectory(sideStart, interiorWaypoints, crossScale, config);
    }

    public void loop(){
        double time = Timer.getFPGATimestamp() - autoStartTime;
        Trajectory.State goal = trajectory.sample(time);
        ChassisSpeeds speeds = ramsete.calculate(driveOdometry.getPoseMeters(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
        curTime = Timer.getFPGATimestamp();
        double dt = curTime - prevTime;
        var targetWheelSpeed = driveKinematics.toWheelSpeeds(ramsete.calculate(pose.get(), trajectory.sample(curTime)));
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;
        double leftFeedforward = motor.driveFeedForward.calculate(leftVelocity, leftVelocity - prevSpeeds.leftMetersPerSecond / dt);
        double rightFeedforward = motor.driveFeedForward.calculate(rightVelocity, rightVelocity - prevSpeeds.rightMetersPerSecond / dt);

        double rightPosition = motor.rightEncoder.getPosition();
        double leftPosition = motor.leftEncoder.getPosition();
        driveOdometry.update(Rotation2d.fromDegrees(motor.ahrs.getFusedHeading() * -1), leftPosition, rightPosition);
        
        motor.setRightVelocity(rightVelocity, rightFeedforward);
        motor.setLeftVelocity(leftVelocity, leftFeedforward);
        
        prevTime = curTime;
        prevSpeeds = targetWheelSpeed;
    }

    public void startAuto(){
        autoStartTime = Timer.getFPGATimestamp();
    }
}
