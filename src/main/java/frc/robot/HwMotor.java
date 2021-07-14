package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
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
	// public final CANSparkMax intake = new CANSparkMax(13, MotorType.kBrushless);
	public final CANPIDController intakePID;
	public final CANSparkMax intake2 = new CANSparkMax(14, MotorType.kBrushless);
	public final CANEncoder intakeEncoder = intake2.getEncoder();
	public final CANSparkMax climberRight = new CANSparkMax(22, MotorType.kBrushless);
	public final CANSparkMax climberLeft = new CANSparkMax(23, MotorType.kBrushless);
	
	public final TalonSRX shooter1 = new TalonSRX(15);
	public final TalonSRX shooter2 = new TalonSRX(16);

	public final CANSparkMax shooterHood = new CANSparkMax(17, MotorType.kBrushless);
	public final CANPIDController hoodPID;
	

	public AHRS ahrs = new AHRS(SerialPort.Port.kUSB); 
	//taken from characterisation tool 1/27/2020
	public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.221, 2.54, 0.513);
	//Track width 62.405 units?


	private double shooter_kP = 0.3;
    private double shooter_kI = 0.0;
    private double shooter_kD = 0.025;
    private double shooter_kF = 0.01;

	private double hood_kP = 0.06;
	private double hood_kF = 0;
	private double hood_kI = 0.0004;
	private double hood_kD = 0;
    
	private double intake_kP = 0;
	private double intake_kF = 0;
	private double intake_kI = 0;
	private double intake_kD = 0;

	//taken from characterisation tool 1/27/2020
	private double leftDrivekP = 0.186;
	private double leftDrivekI = 0;
	private double leftDrivekD = 0;
	private double leftDrivekF = 0;

	//taken from characterisation tool 1/27/2020
	private double rightDrivekP = 0.186;
	private double rightDrivekI = 0;
	private double rightDrivekD = 0;
	private double rightDrivekF = 0;



	public HwMotor(Robot robot) {
		this.robot = robot;
		shooter2.follow(shooter1);
		shooter2.setInverted(InvertType.OpposeMaster);
		shooter1.setSensorPhase(true);
		shooter1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
		shooter1.configVelocityMeasurementWindow(16);
		shooter1.setInverted(true);
		
		shooter1.configNominalOutputForward(0, 30);
		shooter1.configNominalOutputReverse(0, 30);
		shooter1.configPeakOutputForward(1, 30);
        shooter1.configPeakOutputReverse(0, 30);
        shooter1.configPeakCurrentLimit(80);
        shooter2.configPeakCurrentLimit(80);
        // shooter1.configContinuousCurrentLimit();

		shooter1.configVoltageCompSaturation(11.5);
		shooter2.configVoltageCompSaturation(11.5);
		shooter1.enableVoltageCompensation(true);
		
		shooter1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5);
        
        /* Config the Velocity closed loop gains in slot0 */
		shooter1.config_kP(0, shooter_kP, 30);
		shooter1.config_kF(0, shooter_kF, 30);
		shooter1.config_kI(0, shooter_kI, 30);
		shooter1.config_kD(0, shooter_kD, 30); 
		robot.tables.pre_kP.setDouble(shooter_kP);
		robot.tables.pre_kI.setDouble(shooter_kI);
		robot.tables.pre_kD.setDouble(shooter_kD);
		robot.tables.pre_kF.setDouble(shooter_kF);

		intakePID = intake2.getPIDController();
		
		intakePID.setOutputRange(-1, 1);
		intakePID.setP(intake_kP);
		intakePID.setI(intake_kI);
		intakePID.setD(intake_kD);
		intakePID.setFF(intake_kF);
		// intake2.follow(intake, true);

		robot.tables.intake_pre_kP.setDouble(intake_kP);
		robot.tables.intake_pre_kF.setDouble(intake_kI);
		robot.tables.intake_pre_kI.setDouble(intake_kD);
		robot.tables.intake_pre_kD.setDouble(intake_kF);

		left2.follow(left1);
		right2.follow(right1);
		right1.setInverted(true);
        
		leftEncoder.setVelocityConversionFactor(((9.0/84.0) * Units.inchesToMeters(6) * Math.PI / 60.0));
		rightEncoder.setVelocityConversionFactor(((9.0/84.0) * Units.inchesToMeters(6) * Math.PI / 60.0));
		leftEncoder.setPositionConversionFactor(((9.0/84.0) * Units.inchesToMeters(6) * Math.PI));
        rightEncoder.setPositionConversionFactor(((9.0/84.0) * Units.inchesToMeters(6) * Math.PI));

		rightPID.setOutputRange(-1, 1);
		rightPID.setP(rightDrivekP);
		rightPID.setI(rightDrivekI);
		rightPID.setD(rightDrivekD);
		rightPID.setFF(rightDrivekF);
		
		leftPID.setOutputRange(-1, 1);
		leftPID.setP(leftDrivekP);
		leftPID.setI(leftDrivekI);
		leftPID.setD(leftDrivekD);
        leftPID.setFF(leftDrivekF);
        
        left1.enableVoltageCompensation(12);
        left2.enableVoltageCompensation(12);
        right1.enableVoltageCompensation(12);
        right2.enableVoltageCompensation(12);

		robot.tables.leftdrivePrekP.setDouble(leftDrivekP);
		robot.tables.leftdrivePrekF.setDouble(leftDrivekF);
		robot.tables.leftdrivePrekI.setDouble(leftDrivekI);
		robot.tables.leftdrivePrekD.setDouble(leftDrivekD);

		robot.tables.rightdrivePrekP.setDouble(rightDrivekP);
		robot.tables.rightdrivePrekF.setDouble(rightDrivekF);
		robot.tables.rightdrivePrekI.setDouble(rightDrivekI);
		robot.tables.rightdrivePrekD.setDouble(rightDrivekD);

		climberLeft.getEncoder().setPosition(0);
		climberRight.getEncoder().setPosition(0);
		climberLeft.setSoftLimit(SoftLimitDirection.kReverse, 0.5f);
		climberRight.setSoftLimit(SoftLimitDirection.kReverse, 0.5f);

		// climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
		// climberRight.enableSoftLimit(SoftLimitDirection.kReverse, true);

		climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
		climberRight.enableSoftLimit(SoftLimitDirection.kReverse, false);

		intake2.setSmartCurrentLimit(40, 20, 1000);
		
		miniShooter.setSmartCurrentLimit(30);
		hopper.setSmartCurrentLimit(30);
		hopper_infeed.setSmartCurrentLimit(30);

		CPM.setSmartCurrentLimit(30);
		CPM.setIdleMode(IdleMode.kBrake);

		left1.setSmartCurrentLimit(70);
		left2.setSmartCurrentLimit(70);
		right1.setSmartCurrentLimit(70);
		right2.setSmartCurrentLimit(70);

		climberLeft.setSmartCurrentLimit(70);
		climberRight.setSmartCurrentLimit(70);

		//Shooter Hood motor stuff
		shooterHood.setSoftLimit(SoftLimitDirection.kForward, 13f);
		shooterHood.setSoftLimit(SoftLimitDirection.kReverse, 0.5f);
		shooterHood.enableSoftLimit(SoftLimitDirection.kForward, true);
		shooterHood.enableSoftLimit(SoftLimitDirection.kReverse, true);
		shooterHood.setSmartCurrentLimit(20);

		hoodPID = shooterHood.getPIDController(); 
		
		hoodPID.setOutputRange(-1, 1);
		hoodPID.setP(hood_kP);
		hoodPID.setI(hood_kI);
		hoodPID.setD(hood_kD);
		hoodPID.setFF(hood_kF);

		CPM.burnFlash();
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
	}

	public void setLeftVelocity(double velocity, double acceleration){
		double feedforward = driveFeedForward.calculate(velocity, acceleration);
		leftPID.setReference(velocity, ControlType.kVelocity, 0, feedforward);
		robot.tables.leftDriveVelSetpoint.setNumber(velocity);
	}

	public void setRightVelocity(double velocity, double acceleration){
		double feedforward = driveFeedForward.calculate(velocity, acceleration);
		rightPID.setReference(velocity, ControlType.kVelocity, 0, feedforward);
		robot.tables.rightDriveVelSetpoint.setNumber(velocity);
	}

	public void resetHoodPosition(){
		shooterHood.getEncoder().setPosition(0);
		shooterHood.set(0);
	}
}