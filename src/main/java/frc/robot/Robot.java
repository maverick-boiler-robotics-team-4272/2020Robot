package frc.robot; 


import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public Teleop teleop;
	public HwJoystick jstick;
	public Camera camera;
	public HwMotor motor;
	public Shooter shooter;
	public HwPneumatics pneumatics;
	public Climber climber;
	public Intake intake;
	public ColorThing color;
	public Hopper hopper;
	public NewAuto auto;
	public NetworkTables tables;
	public AllBot allbot;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		tables = new NetworkTables(this);
		teleop = new Teleop(this);
		auto = new NewAuto(this);
		motor = new HwMotor(this);
		jstick = new HwJoystick();
		camera = new Camera(this);
		shooter = new Shooter(this);
		pneumatics = new HwPneumatics();
		climber = new Climber(this);
		intake = new Intake(this);
		color = new ColorThing(this);
		hopper = new Hopper(this);
		allbot = new AllBot(this);

		tables.postInit();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use
	 * this for items like diagnostics that you want ran during disabled,
	 * autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		tables.logNetworkTables();
		hopper.update_tables();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
		auto.startAuto();
	
		// auto.generateTrajectory();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		/*switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}*/
		allbot.loopAll();
		auto.loop2();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		teleop.run();
		allbot.loopAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
