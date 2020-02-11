// package frc.robot;
// //import edu.wpi.first.networktables.*;

// public class auto {
//     // private Joystick jstick = new Joystick();
//     private hwControl control = new hwControl();

//     public void drive(double speed){
//         control.drive(speed, speed);
//     }

//     if(jstick.xbox.getYButtonPressed()) {
//         float Kp = -0.1f;
//         float min_command = 0.05f;

//         NetworkTable table = NetworkTable.GetTable("limelight");
//         float tx = table.GetNumber("tx");

//         if (jstick.xbox.getYbuttonReleased()) {
//             float heading_error = -tx;
//             float steering_adjust = 0.0f;
//             if (tx > 1.0) {
//                 steering_adjust = Kp * heading_error - min_command;
//             } else if (tx < 1.0) {
//                 steering_adjust = Kp * heading_error + min_command;
//             }
//             left_command += steering_adjust;
//             right_command -= steering_adjust;
//         }
//     }
// }
