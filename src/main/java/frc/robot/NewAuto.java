/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    Robot robot;
    RamseteController ramsete = new RamseteController();
    DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(62.40537218874971 / 100);
    DifferentialDriveKinematicsConstraint driveContraint = new DifferentialDriveKinematicsConstraint(driveKinematics, 10);
    public DifferentialDriveOdometry driveOdometry;
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(3));
    public double autoStartTime;
    public Trajectory trajectory;
    double RPM_TO_MPS = (9/84) * Units.inchesToMeters(6) * Math.PI;
    DifferentialDriveWheelSpeeds prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
    double prevTime = 0;
    double curTime = 0;


    public NewAuto(Robot robot){
        this.robot = robot;
    }

    public void generateTrajectory() {
        config.addConstraint(driveContraint);
        config.setReversed(false);
        var startWaypoint = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
        var endWaypoint = new Pose2d(Units.feetToMeters(20), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
        var interiorWaypoints = new ArrayList<Translation2d>();
        //interiorWaypoints.add(new Translation2d(Units.feetToMeters(2), Units.feetToMeters(2)));
        // interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.75), Units.feetToMeters(0)));
        trajectory = TrajectoryGenerator.generateTrajectory(startWaypoint, interiorWaypoints, endWaypoint, config);
    }

    public void loop(){
        double time = Timer.getFPGATimestamp() - autoStartTime;

        double rightPosition = robot.motor.rightEncoder.getPosition();
        double leftPosition = robot.motor.leftEncoder.getPosition();
        driveOdometry.update(Rotation2d.fromDegrees(robot.motor.ahrs.getFusedHeading() * -1), leftPosition, rightPosition);

        Trajectory.State goal = trajectory.sample(time);
        ChassisSpeeds speeds = ramsete.calculate(driveOdometry.getPoseMeters(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
        double dt = time - prevTime;
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;
        double leftFeedforward = robot.motor.driveFeedForward.calculate(leftVelocity, (leftVelocity - prevSpeeds.leftMetersPerSecond) / dt);
        double rightFeedforward = robot.motor.driveFeedForward.calculate(rightVelocity, (rightVelocity - prevSpeeds.rightMetersPerSecond) / dt);
        
        robot.motor.setRightVelocity(rightVelocity, rightFeedforward);
        robot.motor.setLeftVelocity(leftVelocity, leftFeedforward);
        
        prevTime = time;
        prevSpeeds = wheelSpeeds;
        System.out.println(goal);
        System.out.println(driveOdometry.getPoseMeters());
        if(driveOdometry.getPoseMeters() == goal.poseMeters){
            robot.pneumatics.CPMSolenoid.set(Value.kForward);
        }
    }

    public void startAuto(){
        robot.motor.leftEncoder.setPosition(0);
        robot.motor.rightEncoder.setPosition(0);
        driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
        autoStartTime = Timer.getFPGATimestamp();
    }

}
