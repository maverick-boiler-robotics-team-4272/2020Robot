package frc.robot;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Hopper {
	//makes sure that hopper can comunicate with the rest of the robot
	Robot robot;

	//gets the line-break sensors from digit pins
	public DigitalInput intake_to_hopper_sense = new DigitalInput(10);
	public DigitalInput hopper_ball_a_sense = new DigitalInput(11);
	public DigitalInput hopper_ball_b_sense = new DigitalInput(12);
	public DigitalInput hopper_ball_c_sense = new DigitalInput(13);
	public DigitalInput shooter_ball_sense = new DigitalInput(14);



	//current sensor values
	public boolean intake_to_hopper_sensor = false;
	public boolean prev_intake_to_hopper_sensor = false;
	public boolean hopper_ball_a = false; // closer to intake
	public boolean hopper_ball_b = false;
	public boolean hopper_ball_c = false; // closer to shooter
	public boolean shooter_ball = false;

	//values so teleop only has to call a function here and the loop function can do the rest
	public boolean is_shooting = false;
	public boolean is_intaking = false;
	public boolean is_stopping = false;
	public boolean is_reversing = false;

	// self explaitory
	int num_balls_in_hopper = 0;

	//makes variables for the motors used here so it is less of a nightmare
	CANSparkMax hopper_infeed;
	CANSparkMax hopper;
	CANSparkMax shooter_infeed;

	//defalt belt speed
	double belt_speed = -0.4;

	// defalt speed for small wheel ebfor shooter fly wheel
	double shooter_feeder_wheel = -0.4;


	//true = error code runs, false = error code not run
	public boolean needCorrections = false;

	//gets the FPGA timestamp of when the intake button is released,
	//so we can leave the intake running 1 second after we released the button.
	private double current_intake_time = 0;

	//gets the double of the target rpm of the shooter for the PID values
	private double rpm = robot.teleop.rpm;

	//instanciates hopper.java and makes sure that we can actually control the hopper
	public Hopper(Robot robot) {
		this.robot = robot;
		hopper_infeed = robot.motor.hopper_infeed;
		hopper = robot.motor.hopper;
		shooter_infeed = robot.motor.miniShooter;
	}

	public void reset(){
		//things to do once
	}
	

	//the loop method that makes sure teleop is less cluttered
	public void loop() {
		// testing for if it is supposed to be shooting
		if(is_shooting){
			movement(true, false);
			robot.shooter.startShooter();
		} else if(is_intaking){ // tests if it supposed to be intaking
			movement(false, false);
		} else if(is_stopping){// tests if it supposed to be stopped
			//movement(false, true);
		} else if(is_reversing) {// tests if it supposed to be in reverse
			hopper_infeed.set(-1 * belt_speed);
			hopper.set(-1 * belt_speed);
			shooter_infeed.set(-1 * shooter_feeder_wheel / 2);
		} else {//if all else fails dont do anything
			disable();
			robot.shooter.stopShooter();
		}


		//if the hopper is intaking or shooting dont compare, but other wise do compare
		if(!is_shooting && !is_intaking){
			if(Timer.getFPGATimestamp() - current_intake_time >= 1){//if one second has passed after intaking has been stopped by operator then stop the hopper
				stop_hopper();
				robot.intake.off();
			}
		}
	}

	public void update_tables(){//updates the network tables for the ball placements
		robot.hopper.readSensorValues(); // update sensor values
		robot.tables.ball1.setBoolean(robot.hopper.intake_to_hopper_sensor);//first ball (closest to intake)
		robot.tables.ball2.setBoolean(robot.hopper.prev_intake_to_hopper_sensor);//second ball (second closest to intake)
		robot.tables.ball3.setBoolean(robot.hopper.hopper_ball_a);//third ball (third closest to intake)
		robot.tables.ball4.setBoolean(robot.hopper.hopper_ball_b);//fourth ball (fourth closest to intake)
		robot.tables.ball5.setBoolean(robot.hopper.hopper_ball_c);//shooter ball (the one right below the shooter fly wheel)
	}

	//the below functions are to make sure loop does not break and so teleop can call it
	public void shoot_balls(){//shoots the balls in the hopper
		is_shooting = true;
		is_intaking = false;
		is_stopping = false;
		is_reversing = false;
	}

	public void intake_balls(){// intakes the ball
		is_shooting = false;
		is_intaking = true;
		is_stopping = false;
		is_reversing = false;
	}

	public void stop_hopper() {//stop all movement in hopper
		is_shooting = false;
		is_intaking = false;
		is_stopping = false;
		is_reversing = false;
	}

	public void reverse_hopper() {//reverses movement in hopper
		is_shooting = false;
		is_intaking = false;
		is_stopping = false;
		is_reversing = true;
	}

	public void stop_intaking(){//starts timer for when the driver stops intaking
		current_intake_time = Timer.getFPGATimestamp();
	}

	private void movement(boolean shoot_button, boolean reverse) {
		double upperDifference = (rpm / Shooter.SENSOR_TO_RPM) * 1.05;//higher end tolerance
		double lowerDifference = (rpm / Shooter.SENSOR_TO_RPM) * 0.95;//the lover end tolerance
		//if(!reverse){
		if (!shoot_button) {//tests if the balls are supposed to be shot out of hopper
			if (!shooter_ball) {//tests if a ball is below the shooter wheel 
				if (intake_to_hopper_sensor && (hopper_ball_a || hopper_ball_b || hopper_ball_c)) {//tests if there is a second ball in hopper
					shooter_infeed.set(shooter_feeder_wheel);//runs wheel just below the shooter fly wheel
					hopper_infeed.set(0);//stops the intake belt so the ball does not go into the second set of belts
					hopper.set(belt_speed);//runs main belt to get ball to the shooter infeed wheel
				} else {//keep moving ball forewards and be ready to intake another ball
					shooter_infeed.set(shooter_feeder_wheel);
					hopper_infeed.set(belt_speed);
					hopper.set(belt_speed);
				}
			} else {//there is a ball right below the shooter fly wheel
				if (intake_to_hopper_sensor) {//tests if there is a ball ready to be put in hopper
					if (!hopper_ball_c) {//not a ball in last spot in hopper
						if (!hopper_ball_b) {//not a ball in second to last spot in hopper
							if (!hopper_ball_a) {//not a ball in the first spot in hopper (2 balls in hopper)
								shooter_infeed.set(0);//dont move ball in from the spot right befor the shooter
								hopper_infeed.set(belt_speed);//move balls in hopper one spot
								hopper.set(belt_speed);//move balls in hopper one spot
							} else {//is a ball in spot a (3 balls in hopper)
								shooter_infeed.set(0);//dont move ball in from the spot right befor the shooter
								hopper_infeed.set(belt_speed);//move balls in hopper one spot
								hopper.set(belt_speed);//move balls in hopper one spot
							}
						} else {//is a ball in spot b (4 balls in hopper)
							shooter_infeed.set(0);//dont move ball in from the spot right befor the shooter
							hopper_infeed.set(belt_speed);//move balls in hopper one spot
							hopper.set(belt_speed);//move balls in hopper one spot
						}
					} else {//is a ball in spot c (5 balls in hopper)
						shooter_infeed.set(0);//move nothing
						hopper_infeed.set(0);//move nothing
						hopper.set(0);//move nothing
					}
				} else {//if there is not a ball in the first set of belts
					if (hopper_ball_a || hopper_ball_b || hopper_ball_c) {//tests for any balls in the main hopper section
						shooter_infeed.set(0);//dont moveball that is just befor shooter fly wheel
						hopper_infeed.set(belt_speed);//move the first set of belts
						hopper.set(0);//but not the main hopper
					} else {//no balls in main hopper area
						shooter_infeed.set(0);//dont moveball that is just befor shooter fly wheel
						hopper_infeed.set(belt_speed);//move first set of belts
						hopper.set(belt_speed);//and main hopper belts
					}
				}
			}
		} else {//shooting balls
			//test if the velocity is within tolerances
			if (robot.motor.shooter1.getSelectedSensorVelocity() >= lowerDifference && robot.motor.shooter1.getSelectedSensorVelocity() <= upperDifference){
				// if it is than move all the belts and shooter infeed wheel
				shooter_infeed.set(-1);
				hopper_infeed.set(-0.2);
				hopper.set(-0.2);
			} else {// if the shooter is not up to speed, then do not move anything
				shooter_infeed.set(0);
				hopper_infeed.set(0);
				hopper.set(0);
			}
		}
	}


	public void readSensorValues() {//reads value from line break sensors, and puts the result in variables
		if (!intake_to_hopper_sense.get()) {
			intake_to_hopper_sensor = true;
		} else {
			intake_to_hopper_sensor = false;
		}
		if (!hopper_ball_a_sense.get()) {
			hopper_ball_a = true;
		} else {
			hopper_ball_a = false;
		}
		if (!hopper_ball_b_sense.get()) {
			hopper_ball_b = true;
		} else {
			hopper_ball_b = false;
		}
		if (!hopper_ball_c_sense.get()) {
			hopper_ball_c = true;
		} else {
			hopper_ball_c = false;
		}
		if (!shooter_ball_sense.get()) {
			shooter_ball = true;
		} else {
			shooter_ball = false;
		}
	}

	private void disable() {//do nothing
		hopper.set(0);
		hopper_infeed.set(0);
		shooter_infeed.set(0);
		// intake_control.off();
	}

	private void errorCorrection(){//error code (not sure what it does)
		if(needCorrections){
			if(!shooter_ball){
				if(!hopper_ball_c){
					if(!hopper_ball_b){
						if(!hopper_ball_a){
							if(!prev_intake_to_hopper_sensor){
								disable();
							}
						}else{
							robot.motor.hopper.set(0.2);
						}
					}else{
						robot.motor.hopper.set(0.2);
					}
				}else if(!hopper_ball_b && hopper_ball_a){
					robot.motor.hopper.set(0.2);
				}else{
					disable();
				}
			}else if(!hopper_ball_a || !hopper_ball_b || !hopper_ball_c){
				if(!prev_intake_to_hopper_sensor){
					robot.motor.hopper_infeed.set(-0.2);//why move it forewards?
				}else{
					robot.motor.hopper_infeed.set(0);
				}
				robot.motor.hopper.set(-0.2);
				robot.motor.miniShooter.set(-0.2);
			}
		}
	}
}
