package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

public class Odometry {
    Robot robot;
    public DifferentialDriveOdometry driveOdometry;
    public Odometry(Robot robot){
        this.robot = robot;
        driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
    }
    public void resetObometry(){
		robot.motor.leftEncoder.setPosition(0);
		robot.motor.rightEncoder.setPosition(0);
        driveOdometry.resetPosition(new Pose2d(),
					Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
    }
    public void update(){
		double rightPosition = robot.motor.rightEncoder.getPosition();
		double leftPosition = robot.motor.leftEncoder.getPosition();
        driveOdometry.update(Rotation2d.fromDegrees(robot.motor.ahrs.getFusedHeading() * -1), leftPosition,
				rightPosition);
    }
}
