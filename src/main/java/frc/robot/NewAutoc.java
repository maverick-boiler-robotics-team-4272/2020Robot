package frc.robot;

import java.io.Console;
import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

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
public class NewAutoc {
	Robot robot;
	RamseteController ramsete = new RamseteController(1.6, 0.7);
	DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(-Units.inchesToMeters(25.5));
	public DifferentialDriveOdometry driveOdometry;
	TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(3));
	public double autoStartTime;
	public Trajectory trajectory = null;
	double RPM_TO_MPS = (9.0 / 84.0) * Units.inchesToMeters(6) * Math.PI;
	DifferentialDriveWheelSpeeds prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
	double prevTime = 0;
	double curTime = 0;
	int state = 0;
	int bouncePathState;
	double timeStampLoop2 = 0;
	double shootCountdown = 0;
	double alignTime = 0;
	double intakeTime = 0;
	Pose2d GoalParams;
	double GoalXTolerance = 0.2;
	double GoalYTolerance = 0.2;
	double GoalAngleTolerance = 90;
	double bouncePathStart = 0;

	public NewAutoc(Robot robot) {
		this.robot = robot;
	}

	// public void pathweaverLoop(){
	// hi
	// }

	/*
	 * public void generateTrajectory() { configConstraints(); var startWaypoint =
	 * new Pose2d(0, 0, Rotation2d.fromDegrees(0)); var endWaypoint = new
	 * Pose2d(Units.feetToMeters(22), 0.0, Rotation2d.fromDegrees(0)); var
	 * interiorWaypoints = List.of( new Translation2d(Units.feetToMeters(5),
	 * Units.feetToMeters(5)), new Translation2d(Units.feetToMeters(10),
	 * Units.feetToMeters(-5)), new Translation2d(Units.feetToMeters(12.5),
	 * Units.feetToMeters(5)), new Translation2d(Units.feetToMeters(16),
	 * Units.feetToMeters(-5)), new Translation2d(Units.feetToMeters(18),
	 * Units.feetToMeters(5)) ); trajectory =
	 * TrajectoryGenerator.generateTrajectory(startWaypoint, interiorWaypoints,
	 * endWaypoint, config); }
	 */

	// Copy of the generateTrajectory function, but using parameters to allow for
	// easier looping for bouncepath
	public void generateTrajectoryWithParameters(boolean reversed, double[][] points) {

		double startX = points[0][0];
		double startY = points[0][1];
		double startAngle = points[0][2];
		double endX = points[(points.length - 1)][0];
		double endY = points[(points.length - 1)][1];
		double endAngle = points[(points.length - 1)][2];
		DifferentialDriveKinematicsConstraint driveContraint = new DifferentialDriveKinematicsConstraint(
				driveKinematics, 3);
		DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(
				robot.motor.driveFeedForward, driveKinematics, 7);
		config.addConstraint(driveContraint);
		config.addConstraint(voltConstraint);
		config.setReversed(reversed);
		var startWaypoint = new Pose2d(Units.feetToMeters(startX), Units.feetToMeters(startY),
				Rotation2d.fromDegrees(startAngle));
		var endWaypoint = new Pose2d(Units.feetToMeters(endX), Units.feetToMeters(endY),
				Rotation2d.fromDegrees(endAngle));
		List<Translation2d> interiorWaypoints = new ArrayList<>();
		if (points.length > 2) {
			for (int i = 1; (points.length - 1) < i; i++) {
				double iX = points[i][0];
				double iY = points[i][1];
				interiorWaypoints.add(new Translation2d(Units.feetToMeters(iX), Units.feetToMeters(iY)));
			}
		} else {
			System.out.println("please have at least 3 points");
			return;
		}
		trajectory = TrajectoryGenerator.generateTrajectory(startWaypoint, interiorWaypoints, endWaypoint, config);
		GoalParams = new Pose2d(Units.feetToMeters(endX), Units.feetToMeters(endY), Rotation2d.fromDegrees(endAngle));
	}

	public void configConstraints() {
		DifferentialDriveKinematicsConstraint driveContraint = new DifferentialDriveKinematicsConstraint(
				driveKinematics, 3);
		DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(
				robot.motor.driveFeedForward, driveKinematics, 7);
		config.addConstraint(driveContraint);
		config.addConstraint(voltConstraint);
		config.setReversed(false);
	}

	public void loop() {
		double time = Timer.getFPGATimestamp() - autoStartTime;

		double rightPosition = robot.motor.rightEncoder.getPosition();
		double leftPosition = robot.motor.leftEncoder.getPosition();
		driveOdometry.update(Rotation2d.fromDegrees(robot.motor.ahrs.getFusedHeading() * -1), leftPosition,
				rightPosition);

		Trajectory.State goal = trajectory.sample(time);
		ChassisSpeeds speeds = ramsete.calculate(driveOdometry.getPoseMeters(), goal);
		DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
		double dt = time - prevTime;
		double leftVelocity = wheelSpeeds.leftMetersPerSecond;
		double rightVelocity = wheelSpeeds.rightMetersPerSecond;
		double rightAcceleration = (rightVelocity - prevSpeeds.rightMetersPerSecond) / dt;
		double leftAcceleration = (leftVelocity - prevSpeeds.rightMetersPerSecond) / dt;
		robot.motor.setRightVelocity(rightVelocity, rightAcceleration);
		robot.motor.setLeftVelocity(leftVelocity, leftAcceleration);

		prevTime = time;
		prevSpeeds = wheelSpeeds;
		// System.out.println("goal: " + goal.poseMeters);
		// System.out.println("real: " + driveOdometry.getPoseMeters());
		// if(driveOdometry.getPoseMeters() == goal.poseMeters){
		// robot.pneumatics.CPMSolenoid.set(Value.kForward);
		// }
	}

	// function to generate the new trajectories based on global variable
	private void newPath() {
		switch (bouncePathState) {
			case 0:
				System.out.println("case 0");
				double[][] pointsa = { { 0, 0, 0 }, { 7, 0 }, { 15, 0, 0 } };
				// int1X, int1Y, int2X, int2Y, endX, endY, endAngle
				generateTrajectoryWithParameters(false, pointsa);
				break;
			// case 1:
			// 	double[][] pointsc = { { 0, 0, 0 }, { 7, 0 }, { 15, 0, 0 } };
			// 	// int1X, int1Y, int2X, int2Y, endX, endY, endAngle
			// 	generateTrajectoryWithParameters(false, pointsc);
			// 	break;
			case 1:
				System.out.println("Hewwo");
				double[][] pointsb = { { 12, 0, 0 }, { 6, 0 }, { 0, 0, 0 } };
				generateTrajectoryWithParameters(true, pointsb);
				break;
			// case 3:
			// 	System.out.println("Hewwo");
			// 	double[][] pointsd = { { 12, 0, 0 }, { 6, 0 }, { 0, 0, 0 } };
			// 	generateTrajectoryWithParameters(true, pointsd);
			// 	break;
		}
	}

	// Bounce Path loop
	public void bouncePath() {
		if (trajectory == null) {
			bouncePathState = 0;
			newPath();
			driveOdometry.resetPosition(trajectory.getInitialPose(),
					Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
		}

		//Test whether the robot is within the tolerance of the end point
		boolean outXTolerance = true;
		boolean outYTolerance = true;
		boolean outAngleTolerance = true;
		if ((((driveOdometry.getPoseMeters().getX()) < GoalParams.getX() + GoalXTolerance)
				&& ((driveOdometry.getPoseMeters().getX()) > GoalParams.getX() - GoalXTolerance))) {
			outXTolerance = false;
		}
		if ((((driveOdometry.getPoseMeters().getY()) < GoalParams.getY() + GoalYTolerance)
				&& ((driveOdometry.getPoseMeters().getY()) > GoalParams.getY() - GoalYTolerance))) {
			outYTolerance = false;
		}
		if ((((driveOdometry.getPoseMeters().getRotation().getDegrees()) < GoalParams.getRotation().getDegrees()
				+ GoalAngleTolerance)
				&& ((driveOdometry.getPoseMeters().getRotation().getDegrees()) > GoalParams.getRotation().getDegrees()
						- GoalAngleTolerance))) {
			outAngleTolerance = false;
		}
		if (!(outXTolerance && outYTolerance && outAngleTolerance)) {
			loop();
		} else {
			System.out.println("bouncePathState pre ++: " + bouncePathState);
			bouncePathState++;
			robot.motor.setRightVelocity(0, 0);
			robot.motor.setLeftVelocity(0, 0);
			newPath();
			// robot.intake.toggle();
			System.out.println("bouncePathState post ++: " + bouncePathState);
			System.out.println("X: " + driveOdometry.getPoseMeters().getX());
			System.out.println("Y: " + driveOdometry.getPoseMeters().getY());
			System.out.println("Goal X: " + GoalParams.getX());
			System.out.println("Goal Y: " + GoalParams.getY());
			// outXTolerance = true;
			// outYTolerance = true;
			// outAngleTolerance = true;
		}
	}

	public void loop2() {
		switch (state) {
			case 0:
				if (robot.hopper.countBalls() != 0) { // this doesn't work because sometimes the balls are in blindspots
					robot.teleop.rpm = 3650;
					robot.shooter.startShooter();
					robot.hopper.shoot_balls();
					shootCountdown = Timer.getFPGATimestamp();
					break;
				} else if (Timer.getFPGATimestamp() > (shootCountdown + 2.0)) {
					robot.shooter.stopShooter();
					robot.hopper.stop_hopper();
					robot.teleop.rpm = 4000;
					state++;
				}
				break;
			case 1:
				timeStampLoop2 = Timer.getFPGATimestamp();
				robot.teleop.drive(0.5, 0.5);
				state++;
			case 2:
				if (Timer.getFPGATimestamp() > timeStampLoop2 + 0.5) {
					robot.teleop.drive(0, 0);
					break;
				}
		}
	}

	public void loop3() {
		switch (state) {
			case 0:
				if (robot.hopper.countBalls() != 0) { // this doesn't work because sometimes the balls are in blindspots
					robot.teleop.rpm = 3650;
					robot.shooter.startShooter();
					robot.hopper.shoot_balls();
					shootCountdown = Timer.getFPGATimestamp();
					break;
				} else if (Timer.getFPGATimestamp() > (shootCountdown + 2.0)) {
					robot.shooter.stopShooter();
					robot.hopper.stop_hopper();
					robot.teleop.rpm = 4000;
					state++;
				}
				break;
			case 1:
				timeStampLoop2 = Timer.getFPGATimestamp();
				robot.teleop.drive(0.5, 0.7);
				state++;
				break;
			case 2:
				if (Timer.getFPGATimestamp() > timeStampLoop2 + 0.5) {
					robot.teleop.drive(0.2, 0.2);
					robot.hopper.intake_balls();
					robot.intake.on(0.5);
					intakeTime = Timer.getFPGATimestamp();
					state++;
				}
				break;
			case 3:
				if (robot.hopper.countBalls() >= 3 || Timer.getFPGATimestamp() > intakeTime + 2) {
					robot.teleop.drive(0, 0);
					// robot.camera.autoAlign();
					robot.hopper.stop_intaking();
					robot.intake.off();
					// state++;
				} else {
					break;
				}
				break;

			// case 4:
			// alignTime = Timer.getFPGATimestamp();
			// state++;
			// break;
			// case 5:
			// if(Timer.getFPGATimestamp() >= alignTime + 1.0 && robot.hopper.countBalls()
			// != 0){
			// // robot.camera.driveVision();
			// robot.hopper.shoot_balls();
			// robot.shooter.startShooter();
			// shootCountdown = Timer.getFPGATimestamp();
			// }else if(robot.hopper.countBalls() == 0 && (Timer.getFPGATimestamp() >
			// shootCountdown + 2)){
			// robot.hopper.stop_hopper();
			// robot.shooter.startShooter();
			// break;
			// }
			// break;
		}
	}

	public void startAuto() {
		robot.motor.leftEncoder.setPosition(0);
		robot.motor.rightEncoder.setPosition(0);
		driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
		// generateTrajectoryWithParameters(false, 1, 0, 3, 0, 5, 0, 0);
		// String trajectoryJSON = "paths/bounce path.wpilib.json";
		// try {
		// Path trajectoryPath =
		// Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
		// trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		// } catch (IOException ex) {
		// DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
		// ex.getStackTrace());
		// }
		// driveOdometry.resetPosition(trajectory.getInitialPose(),Rotation2d.fromDegrees(-1
		// * robot.motor.ahrs.getFusedHeading()));
		autoStartTime = Timer.getFPGATimestamp();
		state = 0;
		robot.intake.toggle();
	}

}
