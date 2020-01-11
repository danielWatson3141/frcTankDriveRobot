/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.Button;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private TalonSRX lTalon;
  private TalonSRX rTalon;
  private Button xButton;

  @Override
  public void robotInit() {
    //m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    lTalon = new WPI_TalonSRX(3);
    rTalon = new WPI_TalonSRX(4);

    //xButton = new Button(1);

  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    //if(xButton.get()){
      //spin();
    //}else{
      lTalon.set(ControlMode.PercentOutput, (m_rightStick.getY()+m_rightStick.getX())*.5);
      rTalon.set(ControlMode.PercentOutput, (-m_rightStick.getY()+m_rightStick.getX())*.5);
    //}
    
    
  }

  //make the robot spin 180 degrees once
  // public void spin() {
  //   //
  //   lTalon.set(ControlMode.PercentOutput, .5);
  //   rTalon.set(ControlMode.PercentOutput, .5);
  // }
}
