/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class colorSensingRobot extends TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final int drive = 0;
  private final int spin = 1;
  private final int spinT = 2;
  private int state = 0;
  private int sectorCount = 0;
  private int startColor;
  private int currentColor;

  private final String red = "RED";
  private final String yellow = "YELLOW";
  private final String blue = "BLUE";
  private final String green = "GREEN";

  private final String[] colors = {red, yellow, blue, green};
  private final String[] LastC = {red};

  private int targetColor = 0;

  private TalonSRX spinnerTalon = new TalonSRX(0);
  XboxController myController = new XboxController(0);

  Color detectedColor;
  double IR;
  double proximity;
 

  @Override
  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    detectedColor = m_colorSensor.getColor();
    SmartDashboard.putString("detected Color", detectedColor.toString());

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    IR = m_colorSensor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putString("Color Sensed", colors[ColorToInt(detectedColor)]);
    SmartDashboard.putString("Target Color", colors[targetColor]);

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
  }

  @Override
  public void teleopPeriodic() {
    int dColor = ColorToInt(detectedColor);

    if(myController.getBumperPressed(Hand.kRight)){
      targetColor += 1;
      if( targetColor > 3){
        targetColor = 0;
      }
    }

    switch (state){
      case drive : 
        drive();
        if(myController.getYButtonPressed()){
          state = spin;
          sectorCount = 0;
          currentColor = dColor;
        }
        else if(myController.getBumperPressed(Hand.kLeft)){
          state = spinT;
          break;
        }

      break;
      case spin : 
        
        spinnerTalon.set(ControlMode.PercentOutput, 1);
        if(myController.getYButtonPressed()){
          state = drive;
          break;
        }
        if(dColor == (currentColor + 1)%4)
        {
          sectorCount++;
          currentColor++;
          currentColor%=4;
        }
        if(sectorCount == 28)
        {
          state = drive;
        }
        
        

      break;
      case spinT :

        

        if(targetColor == dColor){
          spinnerTalon.set(ControlMode.PercentOutput, 0);
          state = drive;
        }else{
          spinnerTalon.set(ControlMode.PercentOutput, 1);
        }
        break;
    }
  }

  private int ColorToInt(Color c){
    double r = c.red;
    double g = c.green;
    double b = c.blue;

    if(r > g && r > b)
      return 0;
    if(g-b > .2){
      if(r > b && g > b)
        return 1;
      return 3;
    }
    return 2;

  }

  private void drive(){
    //drive
  }
}
