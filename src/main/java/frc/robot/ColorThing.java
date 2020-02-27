package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

//   NetworkTableEntry rawColorSensor_r = table.getEntry("rawColorSensor_r");
//   NetworkTableEntry rawColorSensor_g = table.getEntry("rawColorSensor_g");
//   NetworkTableEntry rawColorSensor_b = table.getEntry("rawColorSensor_b");
//   NetworkTableEntry proximit = table.getEntry("proximity");

public class ColorThing {
  Robot robot;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.21, 0.44, 0.3);
  private final Color kGreenTarget = ColorMatch.makeColor(0.2, 0.6, 0.2);
  private final Color kRedTarget = ColorMatch.makeColor(0.67, 0.27, 0.0);
  private final Color kYellowTarget = ColorMatch.makeColor(0.42, 0.51, 0.0);
  private final Color kFakeYellowTarget = ColorMatch.makeColor(0.6, 0.31, 0.05);
  private final Color kFakeGreenTarget = ColorMatch.makeColor(0.3, 0.48, 0.22);

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("datatable");
  NetworkTableEntry colorSensor = table.getEntry("colorSensor");
  NetworkTableEntry rawColorSensor_r = table.getEntry("rawColorSensor_r");
  NetworkTableEntry rawColorSensor_g = table.getEntry("rawColorSensor_g");
  NetworkTableEntry rawColorSensor_b = table.getEntry("rawColorSensor_b");
  NetworkTableEntry proximit = table.getEntry("proximity");
  NetworkTableEntry go_stop = table.getEntry("go_stop");

  private int y_itr = 10;
  private int b_itr = 10; 
  private int g_itr = 10;
  private int r_itr = 10; 
  private int rotations = 8;
  private String current_color;
  private boolean is_spinning;

  public ColorThing(Robot robot) {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kFakeYellowTarget);
    m_colorMatcher.addColorMatch(kFakeGreenTarget);
    this.robot = robot;
    is_spinning = false;
  }

  public void colorRotation(boolean reset) {
    // control panel
    Color detectedColor = m_colorSensor.getColor();
    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else if (match.color == kFakeYellowTarget) {
      colorString = "Fake Yellow";
    } else if (match.color == kFakeGreenTarget) {
      colorString = "Fake Green";
    } else {
      colorString = "Unknown";
    }
    colorSensor.setString(colorString);
    rawColorSensor_r.setDouble(detectedColor.red);
    rawColorSensor_g.setDouble(detectedColor.green);
    rawColorSensor_b.setDouble(detectedColor.blue);
    proximit.setDouble(m_colorSensor.getProximity());

    if (colorString == "Yellow") {
      if (current_color != colorString) {
        current_color = colorString;
        y_itr += 1;
      }
    }
    if (colorString == "Blue") {
      if (current_color != colorString) {
        current_color = colorString;
        b_itr += 1;
      }
    }
    if (colorString == "Green") {
      if (current_color != colorString) {
        current_color = colorString;
        g_itr += 1;
      }
    }
    if (colorString == "Red") {
      if (current_color != colorString) {
        current_color = colorString;
        r_itr += 1;
      }
    }
    if (y_itr >= rotations || b_itr >= rotations || g_itr >= rotations || r_itr >= rotations) {
      go_stop.setString("STOP");
      is_spinning=false;
    } else {
      go_stop.setString("Keep going");
    }
    if (reset) {
      y_itr = 0;
      b_itr = 0;
      g_itr = 0;
      r_itr = 0;
      is_spinning = true;
    }
    
    if(/*is_spinning*/reset){
       robot.motor.CPM.set(0.3);
     } else {
      robot.motor.CPM.set(0);
     }
  }

  public void colorSelection(String color) {
    Color detectedColor = m_colorSensor.getColor();
    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    if(color == "Yellow"){
      if(colorString != "Green"){
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }else if(color == "Blue"){
      if(colorString != "Red"){
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }else if(color == "Red"){
      if(colorString != "Blue"){
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }else if(color == "Green"){
      if(colorString != "Yellow"){ // this was == not !=
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }
  }

  public String colorFromDriveStation(){
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0){
      switch(gameData.charAt(0)){
        case 'B' :
          return "blue";
        case 'G' :
          return "green";
        case 'R' :
          return "red";
        case 'Y' :
          return "yellow";
        default :
          return "undefined";
      }
    }else{
      return "undefined";
    }
  }

  public void doTheColorPosition(){
    if(colorFromDriveStation() == "green"){
      colorSelection("Green");
    }
    if(colorFromDriveStation() == "red"){
      colorSelection("Red");
    }
    if(colorFromDriveStation() == "blue"){
      colorSelection("Blue");
    }
    if(colorFromDriveStation() == "yellow"){
      colorSelection("Yellow");
    }
  }
}