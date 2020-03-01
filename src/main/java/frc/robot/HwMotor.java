/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Units;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;


/**
 * Add your docs here.
 */
public class HwMotor {
    Robot robot;
    public final CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
    public final CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
    public final CANEncoder leftEncoder = left1.getEncoder();
    public final CANPIDController leftPID = left1.getPIDController();
    public final CANSparkMax right1 = new CANSparkMax(3, MotorType.kBrushless);
    public final CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
    public final CANEncoder rightEncoder = right1.getEncoder();
    public final CANPIDController rightPID = right1.getPIDController();
    public final CANSparkMax CPM = new CANSparkMax(20, MotorType.kBrushless);

    public final CANSparkMax hopper_infeed = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax hopper = new CANSparkMax(11, MotorType.kBrushless);
    public final CANSparkMax miniShooter = new CANSparkMax(12, MotorType.kBrushless);
    public final CANSparkMax intake = new CANSparkMax(13, MotorType.kBrushless);
    public final CANPIDController intakePID;
    public final CANEncoder intakeEncoder = intake.getEncoder();
    public final CANSparkMax intake2 = new CANSparkMax(14, MotorType.kBrushless);
    public final CANSparkMax climberRight = new CANSparkMax(22, MotorType.kBrushless);
    public final CANSparkMax climberLeft = new CANSparkMax(23, MotorType.kBrushless);
    
    public final TalonSRX shooter1 = new TalonSRX(15);
    public final TalonSRX shooter2 = new TalonSRX(16);

    public AHRS ahrs = new AHRS(SerialPort.Port.kUSB); 
    public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.211, 0.0295, 0.00628);
    //Track width 62.405 units?

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    public NetworkTableEntry ball1 = table.getEntry("BallAStatus");
    public NetworkTableEntry ball2 = table.getEntry("BallBStatus");
    public NetworkTableEntry ball3 = table.getEntry("BallCStatus");
    public NetworkTableEntry ball4 = table.getEntry("BallDStatus");
    public NetworkTableEntry ball5 = table.getEntry("BallEStatus");

    private double kP = 0;
    private double kF = 0;
    private double kI = 0;
    private double kD = 0;
    private double intake_kP = 0;
    private double intake_kF = 0;
    private double intake_kI = 0;
    private double intake_kD = 0;

    private double drivekP = 0;
    private double drivekI = 0;
    private double drivekD = 0;
    private double drivekF = 0;



    public HwMotor(Robot robot) {
        this.robot = robot;
        shooter2.follow(shooter1);
        shooter2.setInverted(InvertType.OpposeMaster);
        shooter1.setSensorPhase(false);
        shooter1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        shooter1.configVelocityMeasurementWindow(16);
        shooter1.setInverted(false);
        
        shooter1.configNominalOutputForward(0, 30);
		shooter1.configNominalOutputReverse(0, 30);
		shooter1.configPeakOutputForward(1, 30);
		shooter1.configPeakOutputReverse(-1, 30);

		/* Config the Velocity closed loop gains in slot0 */
		shooter1.config_kP(0, kP, 30);
		shooter1.config_kF(0, kF, 30);
		shooter1.config_kI(0, kI, 30);
        shooter1.config_kD(0, kD, 30); 

        shooter1.configVoltageCompSaturation(12);
        // shooter2.configVoltageCompSaturation(12);
        shooter1.enableVoltageCompensation(true);
        
        robot.tables.pre_kP.setDouble(0.15);
        robot.tables.pre_kI.setDouble(0.005);
        robot.tables.pre_kD.setDouble(0.0114);
        robot.tables.pre_kF.setDouble(0.0101);

        
        intakePID = intake.getPIDController();
        
        intakePID.setOutputRange(-1, 1);
        intakePID.setP(intake_kP);
        intakePID.setI(intake_kI);
        intakePID.setD(intake_kD);
        intakePID.setFF(intake_kF);
        // intake2.follow(intake, true);

        robot.tables.intake_pre_kP.setDouble(0);
        robot.tables.intake_pre_kF.setDouble(0.000093);
        robot.tables.intake_pre_kI.setDouble(0);
        robot.tables.intake_pre_kD.setDouble(0);

        left2.follow(left1);
        right2.follow(right1);
        right1.setInverted(true);

        leftEncoder.setVelocityConversionFactor(((9/84) * Units.inchesToMeters(6) * Math.PI / 60) / 42);
        rightEncoder.setVelocityConversionFactor(((9/84) * Units.inchesToMeters(6) * Math.PI / 60) / 42);
        leftEncoder.setPositionConversionFactor(((9/84) * Units.inchesToMeters(6) * Math.PI) / 42);
        rightEncoder.setPositionConversionFactor(((9/84) * Units.inchesToMeters(6) * Math.PI) / 42);

        rightPID.setOutputRange(-1, 1);
        rightPID.setP(drivekP);
        rightPID.setI(drivekI);
        rightPID.setD(drivekD);
        rightPID.setFF(drivekF);
        leftPID.setOutputRange(-1, 1);
        leftPID.setP(drivekP);
        leftPID.setI(drivekI);
        leftPID.setD(drivekD);
        leftPID.setFF(drivekF);

        robot.tables.drivePrekP.setDouble(0);
        robot.tables.drivePrekF.setDouble(0.0);
        robot.tables.drivePrekI.setDouble(0);
        robot.tables.drivePrekD.setDouble(0);

        // climberLeft.follow(climberRight);
        //climberLeft.follow(null);
        // climberLeft.setInverted(true);
        climberLeft.getEncoder().setPosition(0);
        climberRight.getEncoder().setPosition(0);
        climberLeft.setSoftLimit(SoftLimitDirection.kForward, 20);
        climberRight.setSoftLimit(SoftLimitDirection.kForward, 20);

        intake.setSmartCurrentLimit(40, 20, 1000);
        intake2.setSmartCurrentLimit(40, 20, 1000);
        
        miniShooter.setSmartCurrentLimit(30);
        hopper.setSmartCurrentLimit(30);
        hopper_infeed.setSmartCurrentLimit(30);

        CPM.setSmartCurrentLimit(40);

        left1.setSmartCurrentLimit(70);
        left2.setSmartCurrentLimit(70);
        right1.setSmartCurrentLimit(70);
        right2.setSmartCurrentLimit(70);

        climberLeft.setSmartCurrentLimit(70);
        climberRight.setSmartCurrentLimit(70);

        CPM.burnFlash();
        intake.burnFlash();
        intake2.burnFlash();
        climberLeft.burnFlash();
        climberRight.burnFlash();
        miniShooter.burnFlash();
        hopper.burnFlash();
        hopper_infeed.burnFlash();
        left1.burnFlash();
        left2.burnFlash();
        right1.burnFlash();
        right2.burnFlash();


        table.addEntryListener("shooter_kP", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKP = value.getDouble();
                shooter1.config_kP(0, newKP, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("shooter_kF", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKF = value.getDouble();
                shooter1.config_kF(0, newKF, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("shooter_kI", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKI = value.getDouble();
                shooter1.config_kP(0, newKI, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("shooter_kD", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double newKD = value.getDouble();
                shooter1.config_kP(0, newKD, 30);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kP", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKP = value.getDouble();
                intakePID.setP(intakeNewKP);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kF", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKF = value.getDouble();
                intakePID.setFF(intakeNewKF);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kI", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKI = value.getDouble();
                intakePID.setI(intakeNewKI);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("intake_kD", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double intakeNewKD = value.getDouble();
                intakePID.setD(intakeNewKD);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekP", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKP = value.getDouble();
                rightPID.setP(driveNewKP);
                leftPID.setP(driveNewKP);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekI", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKI = value.getDouble();
                rightPID.setI(driveNewKI);
                leftPID.setI(driveNewKI);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekD", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKD = value.getDouble();
                rightPID.setD(driveNewKD);
                leftPID.setD(driveNewKD);
            }
            
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);

        table.addEntryListener("drivekF", new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
                double driveNewKF = value.getDouble();
                rightPID.setFF(driveNewKF);
                leftPID.setFF(driveNewKF);
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

    public void setLeftVelocity(double rpmSetpoint, double feedforward){
        leftPID.setReference(rpmSetpoint, ControlType.kVelocity, 0, feedforward);
        robot.tables.leftDriveVelSetpoint.setNumber(rpmSetpoint);
    }

    public void setRightVelocity(double rpmSetpoint, double feedforward){
        rightPID.setReference(rpmSetpoint, ControlType.kVelocity, 0, feedforward);
        robot.tables.rightDriveVelSetpoint.setNumber(rpmSetpoint);
    }

    public void climberExtend(double power){
        climberRight.set(power);
    }
}