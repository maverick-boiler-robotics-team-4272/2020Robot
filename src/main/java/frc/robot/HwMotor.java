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
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;


/**
 * Add your docs here.
 */
public class HwMotor {
    public final CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
    public final CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
    public final CANSparkMax right1 = new CANSparkMax(3, MotorType.kBrushless);
    public final CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax CPM = new CANSparkMax(20, MotorType.kBrushless);

    public final CANSparkMax hopper_infeed = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax hopper = new CANSparkMax(11, MotorType.kBrushless);
    public final CANSparkMax miniShooter = new CANSparkMax(12, MotorType.kBrushless);
    public final CANSparkMax intake = new CANSparkMax(13, MotorType.kBrushless);
    public final CANPIDController intakePID;
    public final CANEncoder intakeEncoder = intake.getEncoder();
    
    public final TalonSRX shooter1 = new TalonSRX(15);
    public final TalonSRX shooter2 = new TalonSRX(16);
    

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    private NetworkTableEntry pre_kP = table.getEntry("shooter_kP");
    private NetworkTableEntry pre_kF = table.getEntry("shooter_kF");
    private NetworkTableEntry pre_kI = table.getEntry("shooter_kI");
    private NetworkTableEntry pre_kD = table.getEntry("shooter_kD");
    private NetworkTableEntry intake_pre_kP = table.getEntry("intake_kP");
    private NetworkTableEntry intake_pre_kF = table.getEntry("intake_kF");
    private NetworkTableEntry intake_pre_kI = table.getEntry("intake_kI");
    private NetworkTableEntry intake_pre_kD = table.getEntry("intake_kD");
    public NetworkTableEntry shooterVel = table.getEntry("ShooterVelocity");
    public NetworkTableEntry intakeVel = table.getEntry("IntakeVelocity");
    public NetworkTableEntry intakeTemp = table.getEntry("IntakeTemp");
    public NetworkTableEntry shooterOutput = table.getEntry("ShooterOutput");
    public NetworkTableEntry shooterVelSetPoint = table.getEntry("ShooterVelocitySetPoint");

    private double kP = 0;
    private double kF = 0;
    private double kI = 0;
    private double kD = 0;
    private double intake_kP = 0;
    private double intake_kF = 0;
    private double intake_kI = 0;
    private double intake_kD = 0;

    public HwMotor() {
        shooter2.follow(shooter1);
        shooter2.setInverted(InvertType.OpposeMaster);
        shooter1.setSensorPhase(true);
        shooter2.setSensorPhase(true);
        shooter1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        shooter1.configVelocityMeasurementWindow(16);
        
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
        
        pre_kP.setDouble(0.25);
        pre_kF.setDouble(0.0155);
        pre_kI.setDouble(0.001);
        pre_kD.setDouble(0.08);

        
        intakePID = intake.getPIDController();
        
        intakePID.setOutputRange(-1, 1);
        intakePID.setP(intake_kP);
        intakePID.setI(intake_kI);
        intakePID.setD(intake_kD);
        intakePID.setFF(intake_kF);

        intake_pre_kP.setDouble(0);
        intake_pre_kF.setDouble(0.000093);
        intake_pre_kI.setDouble(0);
        intake_pre_kD.setDouble(0);


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



        intake.setSmartCurrentLimit(40, 20, 1000);
        
    }
}
