package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import frc.robot.AutoForFiniteRecharge.typePath;

public class NetworkTables {
	Robot robot;

	NetworkTableInstance inst = NetworkTableInstance.getDefault();

	// standard robot network tables
	NetworkTable table = inst.getTable("datatable");
	//shooter PID
	public NetworkTableEntry pre_kP = table.getEntry("shooter_kP");
	public NetworkTableEntry pre_kF = table.getEntry("shooter_kF");
	public NetworkTableEntry pre_kI = table.getEntry("shooter_kI");
	public NetworkTableEntry pre_kD = table.getEntry("shooter_kD");

	//intake PID
	public NetworkTableEntry intake_pre_kP = table.getEntry("intake_kP");
	public NetworkTableEntry intake_pre_kF = table.getEntry("intake_kF");
	public NetworkTableEntry intake_pre_kI = table.getEntry("intake_kI");
	public NetworkTableEntry intake_pre_kD = table.getEntry("intake_kD");

	//shooter speed readout
	public NetworkTableEntry shooterVel = table.getEntry("ShooterVelocity");

	//intake speed readout
	public NetworkTableEntry intakeVel = table.getEntry("IntakeVelocity");

	//intake temp readout
	public NetworkTableEntry intakeTemp = table.getEntry("IntakeTemp");

	//current readout
	public NetworkTableEntry shooterOutput = table.getEntry("ShooterOutput");

	//other shooter things
	public NetworkTableEntry shooterVelSetPoint = table.getEntry("ShooterVelocitySetPoint");
	public NetworkTableEntry shooterVelHigh = table.getEntry("shooterVelSetpoint");
	public NetworkTableEntry shooterVelLow = table.getEntry("shooterVelLow");

	//left side drive PID
	public NetworkTableEntry leftdrivePrekP = table.getEntry("leftdrivekP");
	public NetworkTableEntry leftdrivePrekI = table.getEntry("leftdrivekI");
	public NetworkTableEntry leftdrivePrekD = table.getEntry("leftdrivekD");
	public NetworkTableEntry leftdrivePrekF = table.getEntry("leftdrivekF");

	//right drive PID
	public NetworkTableEntry rightdrivePrekP = table.getEntry("rightdrivekP");
	public NetworkTableEntry rightdrivePrekI = table.getEntry("rightdrivekI");
	public NetworkTableEntry rightdrivePrekD = table.getEntry("rightdrivekD");
	public NetworkTableEntry rightdrivePrekF = table.getEntry("rightdrivekF");

	//current drive velocity
	public NetworkTableEntry rightDriveVel = table.getEntry("rightDriveVelocity");
	public NetworkTableEntry rightDriveOutput = table.getEntry("rightDriveOutput");
	public NetworkTableEntry rightDriveVelSetpoint = table.getEntry("rightSetpointReadout");
	public NetworkTableEntry leftDriveVel = table.getEntry("leftDriveVelocity");
	public NetworkTableEntry leftDriveOutput = table.getEntry("leftDriveOutput");
	public NetworkTableEntry leftDriveVelSetpoint = table.getEntry("leftSetpointReadout");

	//ball readouts
	public NetworkTableEntry ball1 = table.getEntry("BallAStatus");
	public NetworkTableEntry ball2 = table.getEntry("BallBStatus");
	public NetworkTableEntry ball3 = table.getEntry("BallCStatus");
	public NetworkTableEntry ball4 = table.getEntry("BallDStatus");
	public NetworkTableEntry ball5 = table.getEntry("BallEStatus");

	// limelight network tables
	NetworkTable limelightTable = inst.getTable("limelight");
	public NetworkTableEntry limelightLed = limelightTable.getEntry("ledMode");
	public NetworkTableEntry limelightValidTarget = limelightTable.getEntry("tv");
	public NetworkTableEntry limelightXDegrees = limelightTable.getEntry("tx");
	public NetworkTableEntry limelightYDegrees = limelightTable.getEntry("ty");

	//path tables
	public NetworkTableEntry drivePosX = table.getEntry("posX");
	public NetworkTableEntry drivePosY = table.getEntry("posY");
	public NetworkTableEntry drivePosAngle = table.getEntry("posAngle");
	public NetworkTableEntry driveGoalPosX = table.getEntry("posGoalX");
	public NetworkTableEntry driveGoalPosY = table.getEntry("posGoalY");
	public NetworkTableEntry driveGoalPosAngle = table.getEntry("posGoalAngle");
	public NetworkTableEntry drivePathPos = table.getEntry("posPathNumber");

	//testing the shooter code for distance
	public NetworkTableEntry shooterDistance = table.getEntry("shooterDistance");
	public NetworkTableEntry shooterHoodAngle = table.getEntry("hoodAngle");

	//get current path for auto
	public NetworkTableEntry autoPathState = table.getEntry("autoPathState");

	//hood positions
	public NetworkTableEntry hoodEncoderCounts = table.getEntry("hoodEncoderCounts");
	public NetworkTableEntry hoodSetpoint = table.getEntry("hoodSetpoint");

	public NetworkTableEntry FMSColor = table.getEntry("FMSColor");

	public NetworkTableEntry shooterAngleTune = table.getEntry("shooterAngleTune");

	public NetworkTableEntry alignKPTune = table.getEntry("alignKPTune");
	public NetworkTableEntry alignKITune = table.getEntry("alignKITune");
	public NetworkTableEntry alignKPTErr = table.getEntry("alignKPTErr");
	public NetworkTableEntry alignKITErr = table.getEntry("alignKITErr");
	public NetworkTableEntry alignTX = table.getEntry("alignTX");

	public NetworkTableEntry autoPathSet = table.getEntry("autoPathSet");



	public NetworkTables(Robot robot) {
		this.robot = robot;
	}

	public void postInit() {
		table.addEntryListener("resetOdometry", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				robot.odometry.resetObometry();
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("shooter_kP", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double newKP = value.getDouble();
				robot.motor.shooter1.config_kP(0, newKP, 30);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("shooter_kF", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double newKF = value.getDouble();
				robot.motor.shooter1.config_kF(0, newKF, 30);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("shooter_kI", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double newKI = value.getDouble();
				robot.motor.shooter1.config_kI(0, newKI, 30);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("shooter_kD", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double newKD = value.getDouble();
				robot.motor.shooter1.config_kD(0, newKD, 30);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("intake_kP", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double intakeNewKP = value.getDouble();
				robot.motor.intakePID.setP(intakeNewKP);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("intake_kF", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double intakeNewKF = value.getDouble();
				robot.motor.intakePID.setFF(intakeNewKF);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("intake_kI", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double intakeNewKI = value.getDouble();
				robot.motor.intakePID.setI(intakeNewKI);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("intake_kD", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double intakeNewKD = value.getDouble();
				robot.motor.intakePID.setD(intakeNewKD);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("leftdrivekP", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKP = value.getDouble();
				robot.motor.leftPID.setP(driveNewKP);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("leftdrivekI", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKI = value.getDouble();
				robot.motor.leftPID.setI(driveNewKI);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("leftdrivekD", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKD = value.getDouble();
				robot.motor.leftPID.setD(driveNewKD);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("leftdrivekF", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKF = value.getDouble();
				robot.motor.leftPID.setFF(driveNewKF);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		

		table.addEntryListener("rightdrivekP", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKP = value.getDouble();
				robot.motor.rightPID.setP(driveNewKP);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("rightdrivekI", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKI = value.getDouble();
				robot.motor.rightPID.setI(driveNewKI);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("rightdrivekD", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKD = value.getDouble();
				robot.motor.rightPID.setD(driveNewKD);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("rightdrivekF", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double driveNewKF = value.getDouble();
				robot.motor.rightPID.setFF(driveNewKF);
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
		
		table.addEntryListener("autoPathSet", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				System.out.println("autoPath: " + value);
				switch(value.getString()){
					case "goal":
						robot.auto.currentPath = typePath.PERPTOGOAL;
						break;
					case "trench":
						robot.auto.currentPath = typePath.PARALELLTOCOLOR;
						break;
					case "shoot":
						robot.auto.currentPath = typePath.ONLYSHOOT;
						break;
					case "nothing":
						robot.auto.currentPath = typePath.DONOTHING;
						break;
				}
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);








		table.addEntryListener("shooterAngleTune", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double angle = value.getDouble();
				robot.shooter.shooterAngleTune = angle;
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("alignKPTune", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double KP = value.getDouble();
				robot.camera.alignKP = KP;
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		table.addEntryListener("alignKITune", new TableEntryListener() {
			@Override
			public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
					int flags) {
				double KI = value.getDouble();
				robot.camera.alignKI = KI;
			}

		}, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);



		// table.addEntryListener("shooterVelHigh", new TableEntryListener() {
		// 	@Override
		// 	public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
		// 			int flags) {
		// 		double shooterVelHigh = value.getDouble();
		// 		robot.teleop.rpmHigh = shooterVelHigh;
		// 	}

		// }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

		// table.addEntryListener("shooterVelLow", new TableEntryListener() {
		// 	@Override
		// 	public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
		// 			int flags) {
		// 		double shooterVelLow = value.getDouble();
		// 		robot.teleop.rpmLow =  shooterVelLow;
		// 	}

		// }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
	}

	public void logNetworkTables() {
		intakeVel.setNumber(robot.motor.intakeEncoder.getVelocity());
		intakeTemp.setNumber(robot.motor.intake2.getMotorTemperature());
		rightDriveVel.setNumber(robot.motor.rightEncoder.getVelocity());
		rightDriveOutput.setNumber(robot.motor.right1.getAppliedOutput());
		leftDriveVel.setNumber(robot.motor.leftEncoder.getVelocity());
		leftDriveOutput.setNumber(robot.motor.left1.getAppliedOutput());
		robot.tables.shooterDistance.setNumber(robot.hood.lidarLite.getDistance());
	}

	public void loop() {
		logNetworkTables();
	}

	public void reset() {
		// things to do once
	}
}