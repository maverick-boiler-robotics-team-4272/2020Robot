package frc.robot;


import com.revrobotics.CANSparkMax;

//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;

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

    double rpm;

    public Hopper(Robot robot) {
        this.robot = robot;
        hopper_infeed = robot.motor.hopper_infeed;
        hopper = robot.motor.hopper;
        shooter_infeed = robot.motor.miniShooter;
    }
    

    public void loop(double rpm) {
        if(is_shooting){
            movement(true, false);
        } else if(is_intaking){
            movement(false, false);
        } else if(is_stopping){
            //movement(false, true);
        } else {
            disable();
        }
        this.rpm = rpm;
    }

    //the below functions are to make sure loop does not break and so teleop can call it
    public void shoot_balls(){
        is_shooting = true;
        is_intaking = false;
        is_stopping = false;
    }

    public void intake_balls(){
        is_shooting = false;
        is_intaking = true;
        is_stopping = false;
    }

    public void reverse_balls(){
        is_shooting = false;
        is_intaking = false;
        is_stopping = true;
    }

    public void stop_hopper(){
        is_shooting = false;
        is_intaking = false;
        is_stopping = false;
    }

    

    public void movement(boolean shoot_button, boolean reverse) {
        // readArduino();
        double upperDifference = (rpm / Shooter.SENSOR_TO_RPM) * 1.1;
        double lowerDifference = (rpm / Shooter.SENSOR_TO_RPM) * 0.9;
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

    public void disable() {
        hopper.set(0);
        hopper_infeed.set(0);
        shooter_infeed.set(0);
        // intake_control.off();
    }

    public void errorCorrection(){
        if(needCorrections){
            if(!shooter_ball){
                if(!hopper_ball_c){
                    if(!hopper_ball_b){
                        if(!hopper_ball_a){

                        }
                    }
                }else{
                    if(!hopper_ball_b){
                        if(!hopper_ball_a){
                            if(!intake_to_hopper_sensor){
                                robot.motor.hopper.set(-0.4);
                                robot.motor.hopper_infeed.set(-0.4);
                            }else{
                                robot.motor.hopper_infeed.set(0);
                            }
                        }else{
                        }
                    }
                }
            }else{
                robot.motor.hopper.set(-0.4);
                robot.motor.intake.set(-0.4);
            }
        }
    }
}