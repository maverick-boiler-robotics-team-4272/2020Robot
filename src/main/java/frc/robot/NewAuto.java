package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class NewAuto {
	Robot robot;
	RamseteController ramsete = new RamseteController();
	DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(-Units.inchesToMeters(25.5));
	public DifferentialDriveOdometry driveOdometry;
	TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(5), Units.feetToMeters(3));
	public double autoStartTime;
	public Trajectory trajectory;
	double RPM_TO_MPS = (9.0/84.0) * Units.inchesToMeters(6) * Math.PI;
	DifferentialDriveWheelSpeeds prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
	double prevTime = 0;
	double curTime = 0;
	int state = 0;
	double timeStampLoop2 = 0;


	public NewAuto(Robot robot){
		this.robot = robot;
	}
	public Trajectory trajectoryGeneration(){
		String trajectoryJSON = "path\\Users\\Owner\\Documents\\2020\\PathWeaver\\Games";

		try {
		Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
		Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		return robot.auto.trajectory;
		} catch (IOException ex) {
		DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		return trajectory;
		}
		
	}
	public void generateTrajectory() {
		DifferentialDriveKinematicsConstraint driveContraint = new DifferentialDriveKinematicsConstraint(driveKinematics, 10);
		DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(robot.motor.driveFeedForward, driveKinematics, 7);	
		config.addConstraint(driveContraint);
		config.addConstraint(voltConstraint);
		config.setReversed(true);
		var startWaypoint = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
		var endWaypoint = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(10), Rotation2d.fromDegrees(180));
		var interiorWaypoints = new ArrayList<Translation2d>();
		interiorWaypoints.add(new Translation2d(Units.feetToMeters(-15), Units.feetToMeters(5)));
		// interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.75), Units.feetToMeters(0)));
		trajectory = TrajectoryGenerator.generateTrajectory(startWaypoint, interiorWaypoints, endWaypoint, config);
	}

	public void loop(){
		double time = Timer.getFPGATimestamp() - autoStartTime;

		double rightPosition = robot.motor.rightEncoder.getPosition();
		double leftPosition = robot.motor.leftEncoder.getPosition(); //* ((9.0/84.0) * Units.inchesToMeters(6) * Math.PI);
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
		System.out.println("goal: " + goal.poseMeters);
		System.out.println("real: " + driveOdometry.getPoseMeters());
		if(driveOdometry.getPoseMeters() == goal.poseMeters){
			robot.pneumatics.CPMSolenoid.set(Value.kForward);
		}
	}

	public void loop2(){
		switch(state){
			case 0:
			if(robot.hopper.countBalls() != 0){ // this doesn't work because sometimes the balls are in blindspots
				robot.shooter.startShooter();
				robot.hopper.shoot_balls();
				break;
			}else{
				robot.shooter.stopShooter();
				robot.hopper.stop_hopper();
				state++;
			}
			case 1:
			timeStampLoop2 = Timer.getFPGATimestamp();
			robot.teleop.drive(-0.5, -0.5);
			state++;
			case 2:
			if(Timer.getFPGATimestamp() > timeStampLoop2 + 1){
				robot.teleop.drive(0,0);
				break;
			}
		}
	}

	public void startAuto(){
		robot.motor.leftEncoder.setPosition(0);
		robot.motor.rightEncoder.setPosition(0);
		driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
		autoStartTime = Timer.getFPGATimestamp();
	}

}
