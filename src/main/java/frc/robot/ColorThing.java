package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class ColorThing {
  Robot robot;//gives access to the motors and network tables

  //declares what the color sensor is and that it is on the I2C port
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  //starts matching colors, and has RGB values for the colors on the wheel
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.21, 0.44, 0.3);
  private final Color kGreenTarget = ColorMatch.makeColor(0.2, 0.6, 0.2);
  private final Color kRedTarget = ColorMatch.makeColor(0.67, 0.27, 0.0);
  private final Color kYellowTarget = ColorMatch.makeColor(0.42, 0.51, 0.0);
  private final Color kFakeYellowTarget = ColorMatch.makeColor(0.6, 0.31, 0.05);
  private final Color kFakeGreenTarget = ColorMatch.makeColor(0.3, 0.48, 0.22);


  //declares our network table entries for simplicity
  NetworkTableEntry colorSensor = robot.tables.colorSensor;
  NetworkTableEntry rawColorSensor_r = robot.tables.rawColorSensor_r;
  NetworkTableEntry rawColorSensor_g = robot.tables.rawColorSensor_g;
  NetworkTableEntry rawColorSensor_b = robot.tables.rawColorSensor_b;
  NetworkTableEntry proximit = robot.tables.proximit;
  NetworkTableEntry go_stop = robot.tables.go_stop;

  //how many times the color has been hit on the wheel
  private int y_itr = 10;
  private int b_itr = 10; 
  private int g_itr = 10;
  private int r_itr = 10; 

  //number of times we want the color panel to spin times two
  private int rotations = 8;

  //stores the current color of the wheel
  private String current_color;

  //lets us know if the wheel is supposed to be spinning
  public boolean is_spinning;


  //instanciates this class and makes sure that the code can match to the correct color
  public ColorThing(Robot robot) {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kFakeYellowTarget);
    m_colorMatcher.addColorMatch(kFakeGreenTarget);
    this.robot = robot;

    //when the robot starts up set the CPM to be not spinning
    is_spinning = false;
  }

  public void colorRotation(boolean reset) {
    //gets the current RGB value fo the color sensor
    Color detectedColor = m_colorSensor.getColor();

    //gets the closest atched color according to the RGB values from above
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    
    //makes sure that the color is human readable
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
      //you done screwed up if you get this
      colorString = "Unknown";
    }

    //publish the current color to network tables
    colorSensor.setString(colorString);

    //publish raw RGB values to network tables 
    rawColorSensor_r.setDouble(detectedColor.red);
    rawColorSensor_g.setDouble(detectedColor.green);
    rawColorSensor_b.setDouble(detectedColor.blue);

    //publish how close the color sensor is to the color wheel
    proximit.setDouble(m_colorSensor.getProximity());

    //tests for the current color, then checks if it was changed if it was not then add one to the given color
    if (colorString == "Yellow") {
      if (current_color != colorString) {// tests if the color has changed to this
        current_color = colorString; //sets the color to be this so we do not repeatedly add one to our integer
        y_itr += 1; //if yes then add one to the color
      }
    }
    if (colorString == "Blue") { // same as above
      if (current_color != colorString) {
        current_color = colorString;
        b_itr += 1;
      }
    }
    if (colorString == "Green") { // same as above
      if (current_color != colorString) {
        current_color = colorString;
        g_itr += 1;
      }
    }
    if (colorString == "Red") { // same as above
      if (current_color != colorString) {
        current_color = colorString;
        r_itr += 1;
      }
    }

    //tests if the number of rotation has exceeded the set amount
    if (y_itr >= rotations || b_itr >= rotations || g_itr >= rotations || r_itr >= rotations) {
      //tells the operator that the CPM has stopped
      go_stop.setString("STOP");
      //stops the CPM
      is_spinning=false;
    } else {
      //if it has not reached the target rotations tell the driver that it is still going
      go_stop.setString("Keep going");
    }

    //if the reset signal has been called than set all variables to 0 so we can rerun this code if needbe
    if (reset) {
      y_itr = 0;
      b_itr = 0;
      g_itr = 0;
      r_itr = 0;
      is_spinning = true;
    }

    //spin the CPM if we are good to go
    if(is_spinning) {
       robot.motor.CPM.set(0.3);
     } else {
      robot.motor.CPM.set(0);
     }
  }

  public void colorSelection(String color /*recieved color from FMS*/) {//stage 3 of the match for color selection from FMS

    //gets color
    Color detectedColor = m_colorSensor.getColor();

    //gets closest color from raw RGB values
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    //puts it human readable format
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      //if you get this than you done screwed up
      colorString = "Unknown";
    }

    //uses color string that was passed in
    if(color == "Yellow"){
      if(colorString != "Green"){ //goes to the next two colors and sees if it is true
      // so we know that we have selected the correct color
        robot.motor.CPM.set(0.3); // if not right color move
      }else{
        robot.motor.CPM.set(0); // if it is than stop CPM
      }
    }else if(color == "Blue"){// same as above
      if(colorString != "Red"){
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }else if(color == "Red"){// same as above
      if(colorString != "Blue"){
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }else if(color == "Green"){// same as above
      if(colorString != "Yellow"){
        robot.motor.CPM.set(0.3);
      }else{
        robot.motor.CPM.set(0);
      }
    }
  }

  public String colorFromDriveStation(){//gets color from FMS

    //gets the single letter from FMS to determine color
    String gameData = DriverStation.getInstance().getGameSpecificMessage();

    if(gameData.length() > 0){//tests if there is actuallt anything from FMS
      switch(gameData.charAt(0)){//gets the letter and reterns the full word because it is more human readable
        case 'B' :
          return "blue";
        case 'G' :
          return "green";
        case 'R' :
          return "red";
        case 'Y' :
          return "yellow";
        default : //something weird happened
          return "undefined";
      }
    }else{//nothing there yet
      return "undefined";
    }
  }

  public void doTheColorPosition(){
    //gets color from FMS and puts it into the function colorSelection
    if(colorFromDriveStation() == "green"){//gets color from FMS
      colorSelection("Green");// calls colorSelection
    }
    if(colorFromDriveStation() == "red"){// same as above
      colorSelection("Red");
    }
    if(colorFromDriveStation() == "blue"){// same as above
      colorSelection("Blue");
    }
    if(colorFromDriveStation() == "yellow"){// same as above
      colorSelection("Yellow");
    }
  }
}
