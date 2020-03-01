package frc.robot;

public class NetworkTables{
    Robot robot;
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //standard robot network tables
    NetworkTable table = inst.getTable("datatable");
    public NetworkTableEntry pre_kP = table.getEntry("shooter_kP");
    public NetworkTableEntry pre_kF = table.getEntry("shooter_kF");
    public NetworkTableEntry pre_kI = table.getEntry("shooter_kI");
    public NetworkTableEntry pre_kD = table.getEntry("shooter_kD");
    public NetworkTableEntry intake_pre_kP = table.getEntry("intake_kP");
    public NetworkTableEntry intake_pre_kF = table.getEntry("intake_kF");
    public NetworkTableEntry intake_pre_kI = table.getEntry("intake_kI");
    public NetworkTableEntry intake_pre_kD = table.getEntry("intake_kD");
    public NetworkTableEntry shooterVel = table.getEntry("ShooterVelocity");
    public NetworkTableEntry intakeVel = table.getEntry("IntakeVelocity");
    public NetworkTableEntry intakeTemp = table.getEntry("IntakeTemp");
    public NetworkTableEntry shooterOutput = table.getEntry("ShooterOutput");
    public NetworkTableEntry shooterVelSetPoint = table.getEntry("ShooterVelocitySetPoint");
    public NetworkTableEntry drivePrekP = table.getEntry("drivekP");
    public NetworkTableEntry drivePrekI = table.getEntry("drivekI");
    public NetworkTableEntry drivePrekD = table.getEntry("drivekD");
    public NetworkTableEntry drivePrekF = table.getEntry("drivekF");
    public NetworkTableEntry rightDriveVel = table.getEntry("rightDriveVelocity");
    public NetworkTableEntry rightDriveOutput = table.getEntry("rightDriveOutput");
    public NetworkTableEntry rightDriveVelSetpoint = table.getEntry("rightSetpointReadout");
    public NetworkTableEntry leftDriveVel = table.getEntry("leftDriveVelocity");
    public NetworkTableEntry leftDriveOutput = table.getEntry("leftDriveOutput");
    public NetworkTableEntry leftDriveVelSetpoint = table.getEntry("leftSetpointReadout");
    public NetworkTableEntry ball1 = table.getEntry("BallAStatus");
    public NetworkTableEntry ball2 = table.getEntry("BallBStatus");
    public NetworkTableEntry ball3 = table.getEntry("BallCStatus");
    public NetworkTableEntry ball4 = table.getEntry("BallDStatus");
    public NetworkTableEntry ball5 = table.getEntry("BallEStatus");

    //limelight network tables
    NetworkTable tableB = inst.getTable("limelight")
    public NetworkTableEntry limelightLed = tableB.getEntry("ledMode");
    public NetworkTableEntry limelightValidTarget = tableB.getEntry("tv");
    public NetworkTableEntry limelightXDegrees = tableB.getEntry("tx");
    public NetworkTableEntry limelightYDegrees = tableB.getEntry("ty");


    public NetworkTables(Robot robot){
        this.robot = robot;

    table.addEntryListener("shooter_kP", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKP = value.getDouble();
                robot.motor.shooter1.config_kP(0, newKP, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("shooter_kF", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKF = value.getDouble();
                robot.motor.shooter1.config_kF(0, newKF, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("shooter_kI", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKI = value.getDouble();
                robot.motor.shooter1.config_kP(0, newKI, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("shooter_kD", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKD = value.getDouble();
                robot.motor.shooter1.config_kP(0, newKD, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kP", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKP = value.getDouble();
                robot.motor.intakePID.setP(intakeNewKP);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kF", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKF = value.getDouble();
                robot.motor.intakePID.setFF(intakeNewKF);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kI", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKI = value.getDouble();
                robot.motor.intakePID.setI(intakeNewKI);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kD", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKD = value.getDouble();
                robot.motor.intakePID.setD(intakeNewKD);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekP", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKP = value.getDouble();
                robot.motor.rightPID.setP(driveNewKP);
                robot.motor.leftPID.setP(driveNewKP);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekI", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKI = value.getDouble();
                robot.motor.rightPID.setI(driveNewKI);
                robot.motor.leftPID.setI(driveNewKI);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekD", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKD = value.getDouble();
                robot.motor.rightPID.setD(driveNewKD);
                robot.motor.leftPID.setD(driveNewKD);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekF", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKF = value.getDouble();
                robot.motor.rightPID.setFF(driveNewKF);
                robot.motor.leftPID.setFF(driveNewKF);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("targetVeloc", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double TargetVeloc = value.getDouble();
                robot.teleop.rpm = TargetVeloc;
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
    }

    public void logNetworkTables(){
        robot.tables.intakeVel.setNumber(intakeEncoder.getVelocity());
        robot.tables.intakeTemp.setNumber(intake.getMotorTemperature());
        robot.tables.rightDriveVel.setNumber(rightEncoder.getVelocity());
        robot.tables.rightDriveOutput.setNumber(right1.getAppliedOutput());
        robot.tables.leftDriveVel.setNumber(leftEncoder.getVelocity());
        robot.tables.leftDriveOutput.setNumber(left1.getAppliedOutput());
    }

    public void loop(){
        logNetworkTables();
    }

    public void reset(){
        //things to do once
    }
}