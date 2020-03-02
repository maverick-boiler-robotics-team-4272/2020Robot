import frc.robot;

public class AllBot{
	Robot robot;
	public Allbot(Robot robot){
		this.robot = robot;
	}

	public void loopAll(){
		robot.tables.loop();
		robot.camera.loop();
		robot.shooter.loop();
		robot.climber.loop();
		robot.intake.loop();
		robot.color.loop();
		robot.hopper.loop();
		robot.tables.loop();
	}

	public void resetAll(){
		robot.tables.reset();
		robot.camera.reset();
		robot.shooter.reset();
		robot.climber.reset();
		robot.intake.reset();
		robot.color.reset();
		robot.hopper.reset();
		robot.tables.reset();
	}
}