package frc.robot;

import java.io.Console;
import java.io.IOException;
import java.lang.annotation.Target;
import java.lang.reflect.Array;
import java.nio.file.Path;
import java.security.cert.TrustAnchor;
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
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

public class AutoForFiniteRecharge {
    Robot robot;
    // RamseteController ramsete = new RamseteController(1.6, 0.7);
    RamseteController ramsete = new RamseteController();
    DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(-Units.inchesToMeters(25.5));
    // public DifferentialDriveOdometry driveOdometry;
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(5));
    // TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(4),
    // Units.feetToMeters(2));
    public double autoStartTime = 0;
    public Trajectory trajectory = null;
    double RPM_TO_MPS = (9.0 / 84.0) * Units.inchesToMeters(6) * Math.PI;
    DifferentialDriveWheelSpeeds prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
    double prevTime = 0;
    double curTime = 0;
    int state = 0;

    int compPathState = -1;
    boolean aim = false;
    boolean shoot = false;
    boolean shootStarted = false;
    double timeDelay = 0;
    boolean intake = false;
    boolean setIntake = false;
    boolean turn = false;
    boolean turnClock = true;
    double turnTime = 0;
    double turnTimeStart = -1;
    enum typePath {
        PERPTOGOAL,
        PARALELLTOCOLOR,
        ONLYSHOOT,
        DONOTHING
    }
    typePath currentPath = typePath.PARALELLTOCOLOR;

    double timeStampLoop2 = 0;
    double shootCountdown = 0;
    double alignTime = 0;
    double intakeTime = 0;
    Pose2d GoalParams;
    // double GoalXTolerance = 0.2;
    // double GoalYTolerance = 0.2;
    // double GoalAngleTolerance = 90;
    double GoalXTolerance = 0.4;
    double GoalYTolerance = 0.4;
    double GoalAngleTolerance = 30;
    double bouncePathStart = 0;
    boolean pathsDone = false;

    public AutoForFiniteRecharge(Robot robot) {
        this.robot = robot;
    }

    // Copy of the generateTrajectory function, but using parameters to allow for
    // easier looping for bouncepath
    /**
     * generate the parameter with interior points without angle measurements
     * 
     * @param reversed if the drive direction should be reversed
     * @param points   format: {{startX, startY, startAngle}, {int1X, int2Y}, ...,
     *                 {endX, endY, endAngle}}
     */
    public void generateTrajectoryWithParameters(boolean reversed, double[][] points) {

        // gets its first and last parameters to make the start and end points with
        // directions
        double startX = points[0][0];
        double startY = points[0][1];
        double startAngle = points[0][2];
        double endX = points[(points.length - 1)][0];
        double endY = points[(points.length - 1)][1];
        double endAngle = points[(points.length - 1)][2];
        configConstraints();
        config.setReversed(reversed);
        // we have to use pose2d's because we need to have a starting and ending angle
        var startWaypoint = new Pose2d(Units.feetToMeters(startX), Units.feetToMeters(startY),
                Rotation2d.fromDegrees(startAngle));
        var endWaypoint = new Pose2d(Units.feetToMeters(endX), Units.feetToMeters(endY),
                Rotation2d.fromDegrees(endAngle));
        // creates empty list to allow the loop to add infinite points
        List<Translation2d> interiorWaypoints = new ArrayList<>();
        if (points.length > 2) {
            for (int i = 1; i < (points.length - 1); i++) {
                double iX = points[i][0];
                double iY = points[i][1];
                interiorWaypoints.add(new Translation2d(Units.feetToMeters(iX), Units.feetToMeters(iY)));
            }
        } else {
            // make sure that someone does not just have a starting point and an ending
            // point
            System.out.println("please have at least 3 points");
            return;
        }
        // generate the trajectory and set as a global variable
        trajectory = TrajectoryGenerator.generateTrajectory(startWaypoint, interiorWaypoints, endWaypoint, config);
        // reset time after path is generated to make sure that the path is generated so
        // it does not speed to the next point and freak out
        autoStartTime = Timer.getFPGATimestamp();
        GoalParams = new Pose2d(Units.feetToMeters(endX), Units.feetToMeters(endY), Rotation2d.fromDegrees(endAngle));
    }

    // does the same thing as above but the internal points have to have an angle
    // with it
    /**
     * generate the parameter with interior points without angle measurements
     * 
     * @param reversed if the drive direction should be reversed
     * @param points   format: {{startX, startY, startAngle}, {int1X, int2Y,
     *                 int1Angle}, ..., {endX, endY, endAngle}}
     */
    public void generateTrajectoryWithQuinticParameters(boolean reversed, double[][] points) {
        configConstraints();
        config.setReversed(reversed);
        // it is pose2d instead of translation2d because pose2d includes angles
        List<Pose2d> interiorWaypoints = new ArrayList<>();
        if (points.length > 1) {
            for (int i = 0; i < (points.length); i++) {
                double iX = points[i][0];
                double iY = points[i][1];
                double iAngle = points[i][2];
                interiorWaypoints.add(
                        new Pose2d(Units.feetToMeters(iX), Units.feetToMeters(iY), Rotation2d.fromDegrees(iAngle)));
            }
        } else {
            System.out.println("please have at least 2 points");
            return;
        }
        trajectory = TrajectoryGenerator.generateTrajectory(interiorWaypoints, config);
        autoStartTime = Timer.getFPGATimestamp();
        GoalParams = new Pose2d(Units.feetToMeters(points[(points.length - 1)][0]),
                Units.feetToMeters(points[(points.length - 1)][1]),
                Rotation2d.fromDegrees(points[(points.length - 1)][2]));
    }

    /**
     * used to have the params for limits with the ramsette auto
     */
    public void configConstraints() {
        // DifferentialDriveKinematicsConstraint driveContraint = new
        // DifferentialDriveKinematicsConstraint(
        // driveKinematics, 4);

        DifferentialDriveKinematicsConstraint driveContraint = new DifferentialDriveKinematicsConstraint(
                driveKinematics, 10);

        // commented this out because of the error of the min acceleration being greater
        // than its max acceleration and this fixed it
        // DifferentialDriveVoltageConstraint voltConstraint = new
        // DifferentialDriveVoltageConstraint(
        // robot.motor.driveFeedForward, driveKinematics, 10);

        // set its max turning acceleration so it does not freak out and overshoot as
        // much
        CentripetalAccelerationConstraint maxTurnAcceleration = new CentripetalAccelerationConstraint(2);
        config.addConstraint(driveContraint);
        // config.addConstraint(voltConstraint);
        config.addConstraint(maxTurnAcceleration);
    }

    /**
     * used to calculate how the robot should move using ramsette
     */
    public void loop() {
        double time = Timer.getFPGATimestamp() - autoStartTime;

        Trajectory.State goal = trajectory.sample(time);
        ChassisSpeeds speeds = ramsete.calculate(robot.odometry.driveOdometry.getPoseMeters(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
        double dt = time - prevTime;
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;
        double rightAcceleration = (rightVelocity - prevSpeeds.rightMetersPerSecond) / dt;
        // double leftAcceleration = (leftVelocity - prevSpeeds.rightMetersPerSecond) /
        // dt;
        double leftAcceleration = (leftVelocity - prevSpeeds.leftMetersPerSecond) / dt;
        robot.motor.setRightVelocity(rightVelocity, rightAcceleration);
        robot.motor.setLeftVelocity(leftVelocity, leftAcceleration);

        prevTime = time;
        prevSpeeds = wheelSpeeds;
    }

    private void newGoalPath() {
        // from the front bumper to the navX in inches divided by 12 to get feet
        double frontToNavX = (18 / 12);
        // some good points Y points for the robot
        // last point is the position so the navX hits the point
        double[][] pointsa = { { 0, 0, 0 }, { 0, 0 }, { 0, 1, 0 } };
        double[][] pointsb = { pointsa[pointsa.length - 1], { 2.5 - frontToNavX, 2.5 },
                { 3 - frontToNavX, 5.33, 0 } };
        double[][] pointsc = { pointsb[pointsb.length-1], { 12 - frontToNavX, 5.33 }, {18 - frontToNavX, 5.33, 0} };
        double[][] pointsd = { pointsc[pointsc.length-1], { 10, 5 }, { 0, 0, 0} };
        // double[][] pointsb = { pointsa[pointsa.length], {}};
        // create a new path from the above parameters (all measurements in feet and
        // angles)
        switch (compPathState) {
        case 0:
            generateTrajectoryWithParameters(false, pointsa);
            break;
        case 1:
            shoot = true;
            intake = true;
            setIntake = false;
            robot.climber.down();
            break;
        case 2:
            generateTrajectoryWithParameters(false, pointsb);
            break;
        case 3:
            config = new TrajectoryConfig(Units.feetToMeters(3), Units.feetToMeters(2));
            generateTrajectoryWithParameters(false, pointsc);
            break;
        case 4:
            robot.hopper.stop_hopper();
            break;
        case 5:
            intake = false;
            robot.intake.off();
            config = new TrajectoryConfig(Units.feetToMeters(14), Units.feetToMeters(10));
            robot.shooter.setShooterRPM(2900);
            generateTrajectoryWithParameters(true, pointsd);
            break;
        case 6:
            aim = true;
            break;
        case 7:
            shoot = true;
            break;
        default:
            // tells the code it is done so it does not increment the path to infinity (and
            // save resources in doing so)
            robot.hopper.stop_hopper();
            pathsDone = true;
        }
        // put all of the goal parameters onto the network tables
        robot.tables.driveGoalPosX.setDouble(GoalParams.getX());
        robot.tables.driveGoalPosY.setDouble(GoalParams.getY());
        robot.tables.driveGoalPosAngle.setDouble(GoalParams.getRotation().getDegrees());
        robot.tables.drivePathPos.setNumber(compPathState);
    }

    private void newColorPath() {
        // from the front bumper to the navX in inches divided by 12 to get feet
        double frontToNavX = (18 / 12);
        // some good points Y points for the robot
        // last point is the position so the navX hits the point
        double[][] pointsa = { { 0, 0, 0 }, { 0, 0 }, { 0, 1, 30 } };
        double[][] pointsb = { { pointsa[pointsa.length - 1][0], pointsa[pointsa.length - 1][1], 0 }, { 2.5 - frontToNavX, 2.5 },
                { 7 - frontToNavX, 2.5, 0 } };
        double[][] pointsc = { pointsb[pointsb.length-1], { 12 - frontToNavX, 2.5 }, {21 - frontToNavX, 2.5, 0} };
        double[][] pointsd = { pointsc[pointsc.length-1], { 10, 0 }, { 0, 0, 0} };
        // double[][] pointsb = { pointsa[pointsa.length], {}};
        // create a new path from the above parameters (all measurements in feet and
        // angles)
        switch (compPathState) {
        case 0:
            generateTrajectoryWithParameters(false, pointsa);
            break;
        case 1:
            aim = true;
            shoot = true;
            intake = true;
            setIntake = false;
            robot.climber.down();
            break;
        case 2:
            generateTrajectoryWithParameters(false, pointsb);
            break;
        case 3:
            config = new TrajectoryConfig(Units.feetToMeters(3), Units.feetToMeters(2));
            generateTrajectoryWithParameters(false, pointsc);
            break;
        case 4:
            robot.hopper.stop_hopper();
            break;
        case 5:
            intake = false;
            robot.intake.off();
            robot.intake.in();
            config = new TrajectoryConfig(Units.feetToMeters(14), Units.feetToMeters(10));
            robot.shooter.setShooterRPM(2900);
            generateTrajectoryWithParameters(true, pointsd);
            break;
        case 6:
            aim = true;
            break;
        case 7:
            shoot = true;
            break;
        default:
            // tells the code it is done so it does not increment the path to infinity (and
            // save resources in doing so)
            robot.hopper.stop_hopper();
            pathsDone = true;
        }
        // put all of the goal parameters onto the network tables
        robot.tables.driveGoalPosX.setDouble(GoalParams.getX());
        robot.tables.driveGoalPosY.setDouble(GoalParams.getY());
        robot.tables.driveGoalPosAngle.setDouble(GoalParams.getRotation().getDegrees());
        robot.tables.drivePathPos.setNumber(compPathState);
    }

    /**
     * done to prevent the creating of a path every loop and save the CPU cycles
     */
    private void incrementCompPath() {
        if(!(compPathState>=0)){
            compPathState = 0;
            switch(currentPath){
                case PERPTOGOAL:
                    newGoalPath();
                    break;
                case PARALELLTOCOLOR:
                    newColorPath();
                    break;
            }
            return;
        }
        if (!pathsDone) {
            System.out.println("slalomPathState pre ++: " + compPathState);
            compPathState++;
            robot.motor.setRightVelocity(0, 0);
            robot.motor.setLeftVelocity(0, 0);
            switch(currentPath){
                case PERPTOGOAL:
                    newGoalPath();
                    break;
                case PARALELLTOCOLOR:
                    newColorPath();
                    break;
            }
            System.out.println("slalomPathState post ++: " + compPathState);
        }
    }

    public void compPath() {
        if (trajectory == null) {
            incrementCompPath();
            robot.odometry.driveOdometry.resetPosition(trajectory.getInitialPose(),
                    Rotation2d.fromDegrees(-1 * robot.motor.ahrs.getFusedHeading()));
        }
        // if(turn){
        //     if(turnClock){
        //         robot.teleop.drive(-0.2, 0.2);
        //     }
        // }
        if (aim) {
            if (robot.camera.updateLimelightTracking(true)) {
                aim = false;
                incrementCompPath();
                return;
            } else {
                robot.camera.updateLimelightTracking(true);
                robot.teleop.drive(-robot.camera.alignTurnSpeed, robot.camera.alignTurnSpeed);
                return;
            }
        }
        if (shoot) {
            if (!shootStarted) {
                timeDelay = Timer.getFPGATimestamp();
                shootStarted = true;
                robot.hopper.shoot_balls();
                robot.shooter.startShooter();
                return;
            } else {
                if (robot.hopper.countBalls() == 0) {
                    if ((Timer.getFPGATimestamp()-timeDelay) > 1) {
                        shoot = false;
                        shootStarted = false;
                        robot.hopper.stop_hopper();
                        robot.shooter.stopShooter();
                        incrementCompPath();
                        return;
                    } else {
                        return;
                    }
                } else {
                    timeDelay = Timer.getFPGATimestamp();
                    return;
                }
            }
        }
        if(intake){
            if(!setIntake){
                robot.intake.out();
                robot.intake.extended = true;
                incrementCompPath();
                setIntake = true;
            }
            robot.hopper.intake_balls();
            robot.intake.on(-0.7);
        }


        // Test whether the robot is within the tolerance of the end point
        boolean outXTolerance = (((robot.odometry.driveOdometry.getPoseMeters().getX()) > GoalParams.getX()
                + GoalXTolerance)
                || ((robot.odometry.driveOdometry.getPoseMeters().getX()) < GoalParams.getX() - GoalXTolerance));
        boolean outYTolerance = (((robot.odometry.driveOdometry.getPoseMeters().getY()) > GoalParams.getY()
                + GoalYTolerance)
                || ((robot.odometry.driveOdometry.getPoseMeters().getY()) < GoalParams.getY() - GoalYTolerance));
        boolean outAngleTolerance = (((robot.odometry.driveOdometry.getPoseMeters().getRotation()
                .getDegrees()) > GoalParams.getRotation().getDegrees() + GoalAngleTolerance)
                || ((robot.odometry.driveOdometry.getPoseMeters().getRotation().getDegrees()) < GoalParams.getRotation()
                        .getDegrees() - GoalAngleTolerance));

        // if within tolerance increment path number the loop otherwise do drive loop
        if ((!outXTolerance && !outYTolerance && !outAngleTolerance)) {
            incrementCompPath();
        } else {
            loop();
        }
    }

    public void startAuto() {
        robot.motor.leftEncoder.setPosition(0);
        robot.motor.rightEncoder.setPosition(0);
        robot.odometry.resetObometry();
        autoStartTime = Timer.getFPGATimestamp();
        state = 0;
        // robot.intake.toggle();
    }
}
