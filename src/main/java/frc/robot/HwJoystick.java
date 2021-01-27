package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class HwJoystick {
	//tank drive controlls for driver
	public final Joystick leftJoystick = new Joystick(1);
	public final Joystick rightJoystick = new Joystick(0);

	//xbox controller for operator
	public final XboxController xbox = new XboxController(2);
}
