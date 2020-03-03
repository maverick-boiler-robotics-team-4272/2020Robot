package frc.robot;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Hopper {
    // public DigitalInput intake_sense = new DigitalInput(0);
    public DigitalInput intake_to_hopper_sense = new DigitalInput(10);
    public DigitalInput hopper_ball_a_sense = new DigitalInput(11);
    public DigitalInput hopper_ball_b_sense = new DigitalInput(12);
    public DigitalInput hopper_ball_c_sense = new DigitalInput(13);
    public DigitalInput shooter_ball_sense = new DigitalInput(14);



    public boolean intake_to_hopper_sensor = false;
    public boolean prev_intake_to_hopper_sensor = false;
    public boolean hopper_ball_a = false; // closer to intake
    public boolean hopper_ball_b = false;
    public boolean hopper_ball_c = false; // closer to shooter
    public boolean shooter_ball = false;

    public boolean is_shooting = false;
    public boolean is_intaking = false;
    public boolean is_stopping = false;
    public boolean is_reversing = false;

    // private I2C Wire;
    // private static final int MAX_BYTES = 32;
    // private int lightState;

    int num_balls_in_hopper = 0;

    Robot robot;

    CANSparkMax hopper_infeed;
    CANSparkMax hopper;
    CANSparkMax shooter_infeed;
    double belt_speed = -0.4;
    double shooter_feeder_wheel = -0.4;
    boolean intakeSwitch = true;
    public boolean needCorrections = false;

    private double current_intake_time = 0;

    double rpm;

    public Hopper(Robot robot) {
        this.robot = robot;
        hopper_infeed = robot.motor.hopper_infeed;
        hopper = robot.motor.hopper;
        shooter_infeed = robot.motor.miniShooter;
    }
    

    public void loop() {
        if(is_shooting){
            movement(true, false);
        } else if(is_intaking){
            movement(false, false);
        } else if(is_stopping){
            //movement(false, true);
        } else if(is_reversing) {
            hopper_infeed.set(-1 * belt_speed);
            hopper.set(-1 * belt_speed);
            shooter_infeed.set(-1 * shooter_feeder_wheel / 2);
        } else {
            disable();
        }
        this.rpm = robot.teleop.rpm;
        if(!is_shooting || !is_intaking){
            if(Timer.getFPGATimestamp() - current_intake_time >= 1){
                stop_hopper();
            }
        }
    }

    public void reset() {}

    public void update_tables(){
        robot.hopper.readArduino(); // update sensor values
        robot.motor.ball1.setBoolean(robot.hopper.intake_to_hopper_sensor);
        robot.motor.ball2.setBoolean(robot.hopper.prev_intake_to_hopper_sensor);
        robot.motor.ball3.setBoolean(robot.hopper.hopper_ball_a);
        robot.motor.ball4.setBoolean(robot.hopper.hopper_ball_b);
        robot.motor.ball5.setBoolean(robot.hopper.hopper_ball_c);
    }

    //the below functions are to make sure loop does not break and so teleop can call it
    public void shoot_balls(){
        is_shooting = true;
        is_intaking = false;
        is_stopping = false;
        is_reversing = false;
    }

    public void intake_balls(){
        is_shooting = false;
        is_intaking = true;
        is_stopping = false;
        is_reversing = false;
    }

    public void stop_hopper() {
        is_shooting = false;
        is_intaking = false;
        is_stopping = false;
        is_reversing = false;
    }

    public void reverse_hopper() {
        is_shooting = false;
        is_intaking = false;
        is_stopping = false;
        is_reversing = true;
    }

    public void stop_intaking(){
        current_intake_time = Timer.getFPGATimestamp();
    }

    private void movement(boolean shoot_button, boolean reverse) {
        // readArduino();
        double upperDifference = (rpm / Shooter.SENSOR_TO_RPM) * 1.05;
        double lowerDifference = (rpm / Shooter.SENSOR_TO_RPM) * 0.95;
        //if(!reverse){
        if (!shoot_button) {
            if (!shooter_ball) {
                if (intake_to_hopper_sensor && (hopper_ball_a || hopper_ball_b || hopper_ball_c)) {
                    shooter_infeed.set(shooter_feeder_wheel);
                    hopper_infeed.set(0);
                    hopper.set(belt_speed);
                    // intake_control.off();
                } else {
                    shooter_infeed.set(shooter_feeder_wheel);
                    hopper_infeed.set(belt_speed);
                    hopper.set(belt_speed);
                    // intake_control.on();
                }
            } else {
                if (intake_to_hopper_sensor) {
                    if (!hopper_ball_c) {
                        if (!hopper_ball_b) {
                            if (!hopper_ball_a) {
                                shooter_infeed.set(0);
                                hopper_infeed.set(belt_speed);
                                hopper.set(belt_speed);
                                // intake_control.on();
                            } else {
                                shooter_infeed.set(0);
                                hopper_infeed.set(belt_speed);
                                hopper.set(belt_speed);
                                // intake_control.on();
                            }
                        } else {
                            shooter_infeed.set(0);
                            hopper_infeed.set(belt_speed);
                            hopper.set(belt_speed);
                            // intake_control.on();
                        }
                    } else {
                        shooter_infeed.set(0);
                        hopper_infeed.set(0);
                        hopper.set(0);
                        // intake_control.off();
                    }
                } else {
                    if (hopper_ball_a || hopper_ball_b || hopper_ball_c) {
                        shooter_infeed.set(0);
                        hopper_infeed.set(belt_speed);
                        hopper.set(0);
                        // intake_control.on();
                    } else {
                        shooter_infeed.set(0);
                        hopper_infeed.set(belt_speed);
                        hopper.set(belt_speed);
                        // intake_control.on();
                    }
                }
            }
        } else {
            if (robot.motor.shooter1.getSelectedSensorVelocity() >= lowerDifference && robot.motor.shooter1.getSelectedSensorVelocity() <= upperDifference){
                shooter_infeed.set(-1);
                hopper_infeed.set(-0.2);
                hopper.set(-0.2);
            } else {
                shooter_infeed.set(0);
                hopper_infeed.set(0);
                hopper.set(0);
            }
        }/*} else {
            retract();
        }*/
    }

    public void retract(){
        hopper_infeed.set(-1 * belt_speed);
        hopper.set(-1 * belt_speed);
    }

    public void readArduino() {
        // byte[] buffer = new byte[MAX_BYTES];
        // byte[] data = new byte[1];
        // data[0] = (byte)lightState;
        // Wire.read(4, MAX_BYTES, buffer);
        // String out = new String(buffer);
        // int pt = out.indexOf((char)255);
        // String num = new String(out.substring(0, pt < 0 ? 0 : pt));
        // System.out.println("received: " + num);
        // int intOut = Integer.parseInt(num);
        // // for(var i = 0; i < 7; i++){
        // // if(intOut % 1 == 1){
        // // hopper_ball_a = true;
        // // }
        // // else if(intOut % 2 == 0){
        // // hopper_ball_b = true;
        // // }
        // // }
        // if(intOut % 2 == 1){
        // intake_sensor = true;
        // } else {
        // intake_sensor = false;
        // }
        // intOut = intOut/2;
        // if(intOut % 2 == 1){
        // intake_to_hopper_sensor = true;
        // } else {
        // intake_to_hopper_sensor= false;
        // }
        // intOut = intOut/2;
        // if(intOut % 2 == 1){
        // hopper_ball_a = true;
        // } else {
        // hopper_ball_a = false;
        // }
        // intOut = intOut/2;
        // if(intOut % 2 == 1){
        // hopper_ball_b = true;
        // } else {
        // hopper_ball_b = false;
        // }
        // intOut = intOut/2;
        // if(intOut % 2 == 1){
        // hopper_ball_c = true;
        // } else {
        // hopper_ball_c = false;
        // }
        // intOut = intOut/2;
        // if(intOut % 2 == 1){
        // shooter_ball = true;
        // } else {
        // shooter_ball = false;
        // }
        // if(intake_sense.get()){
        // intake_sensor = true;
        // } else {
        // intake_sensor = false;
        // }
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

        /*
        System.out.println("intake_to_hopper_sensor : " + intake_to_hopper_sensor + "\n hopper_ball_a : "
                + hopper_ball_a + "\n hopper_ball_b : " + hopper_ball_b + "\n hopper_ball_c : " + hopper_ball_c
                + "\n shooter_ball : " + shooter_ball);
        */
    }

    private void disable() {
        hopper.set(0);
        hopper_infeed.set(0);
        shooter_infeed.set(0);
        // intake_control.off();
    }

    private void errorCorrection() {
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
                    robot.motor.hopper_infeed.set(-0.2);
                }else{
                    robot.motor.hopper_infeed.set(0);
                }
                robot.motor.hopper.set(-0.2);
                robot.motor.miniShooter.set(-0.2);
            }
        }
    }
}
