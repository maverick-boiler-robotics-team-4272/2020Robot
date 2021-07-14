package frc.robot;

public class AllBot{
	Robot robot;
	public AllBot(Robot robot){
		this.robot = robot;
	}

	/**
	 * loop all of the classes that we use
	 */
	public void loopAll(){
		robot.tables.loop();
		robot.camera.loop();
		robot.shooter.loop();
		robot.climber.loop();
		robot.intake.loop();
		// robot.color.loop();
		robot.hopper.loop();
		robot.hood.loop();
	}

	/**
	 * reset all of the classes that we use
	 */
	public void resetAll(){
		robot.tables.reset();
		robot.camera.reset();
		robot.shooter.reset();
		robot.climber.reset();
		robot.intake.reset();
		// robot.color.reset();
		robot.hopper.reset();
	}
}