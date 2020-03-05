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
	public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.214/12, 33.9/12, 6.55/12);
	//Track width 62.405 units?

	NetworkTableInstance inst = NetworkTableInstance.getDefault();
	NetworkTable table = inst.getTable("datatable");
	public NetworkTableEntry ball1 = table.getEntry("BallAStatus");
	public NetworkTableEntry ball2 = table.getEntry("BallBStatus");
	public NetworkTableEntry ball3 = table.getEntry("BallCStatus");
	public NetworkTableEntry ball4 = table.getEntry("BallDStatus");
	public NetworkTableEntry ball5 = table.getEntry("BallEStatus");

	private double shooter_kP = 0.3;
    private double shooter_kI = 0.0;
    private double shooter_kD = 0.025;
    private double shooter_kF = 0.01;
    
	private double intake_kP = 0;
	private double intake_kF = 0;
	private double intake_kI = 0;
	private double intake_kD = 0;

	private double drivekP = 1.19;
	private double drivekI = 0;
	private double drivekD = 0;
	private double drivekF = 0;



	public HwMotor(Robot robot) {
		this.robot = robot;
		shooter2.follow(shooter1);
		shooter2.setInverted(InvertType.FollowMaster);
		shooter1.setSensorPhase(false);
		shooter1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
		shooter1.configVelocityMeasurementWindow(16);
		shooter1.setInverted(false);
		
		shooter1.configNominalOutputForward(0, 30);
		shooter1.configNominalOutputReverse(0, 30);
		shooter1.configPeakOutputForward(1, 30);
        shooter1.configPeakOutputReverse(0, 30);
        shooter1.configPeakCurrentLimit(80);
        shooter2.configPeakCurrentLimit(80);
        // shooter1.configContinuousCurrentLimit();

		shooter1.configVoltageCompSaturation(12);
		// shooter2.configVoltageCompSaturation(12);
        shooter1.enableVoltageCompensation(true);
        
        /* Config the Velocity closed loop gains in slot0 */
		shooter1.config_kP(0, shooter_kP, 30);
		shooter1.config_kF(0, shooter_kF, 30);
		shooter1.config_kI(0, shooter_kI, 30);
		shooter1.config_kD(0, shooter_kD, 30); 
		robot.tables.pre_kP.setDouble(shooter_kP);
		robot.tables.pre_kI.setDouble(shooter_kI);
		robot.tables.pre_kD.setDouble(shooter_kD);
		robot.tables.pre_kF.setDouble(shooter_kF);

		intakePID = intake.getPIDController();
		
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
		rightPID.setP(drivekP);
		rightPID.setI(drivekI);
		rightPID.setD(drivekD);
		rightPID.setFF(drivekF);
		
		leftPID.setOutputRange(-1, 1);
		leftPID.setP(drivekP);
		leftPID.setI(drivekI);
		leftPID.setD(drivekD);
        leftPID.setFF(drivekF);
        
        left1.enableVoltageCompensation(12);
        left2.enableVoltageCompensation(12);
        right1.enableVoltageCompensation(12);
        right2.enableVoltageCompensation(12);

		robot.tables.drivePrekP.setDouble(drivekP);
		robot.tables.drivePrekF.setDouble(drivekF);
		robot.tables.drivePrekI.setDouble(drivekI);
		robot.tables.drivePrekD.setDouble(drivekD);

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

		CPM.setSmartCurrentLimit(30);

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
	}

	public void setLeftVelocity(double rpmSetpoint, double feedforward){
		leftPID.setReference(rpmSetpoint, ControlType.kVelocity, 0, feedforward);
		robot.tables.leftDriveVelSetpoint.setNumber(rpmSetpoint);
	}

	public void setRightVelocity(double rpmSetpoint, double feedforward){
		rightPID.setReference(rpmSetpoint, ControlType.kVelocity, 0, feedforward);
		robot.tables.rightDriveVelSetpoint.setNumber(rpmSetpoint);
	}
}