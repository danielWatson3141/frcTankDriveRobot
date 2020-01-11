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
import edu.wpi.first.wpilibj.buttons.*;

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
  private TalonSRX l1Talon;
  private TalonSRX r1Talon;
  private TalonSRX l2Talon;
  private TalonSRX r2Talon;
  private JoystickButton aButton;
  private JoystickButton bButton;
  private JoystickButton xButton;
  private JoystickButton yButton;
  private JoystickButton lBumper;
  private JoystickButton rBumper;
  private JoystickButton backButton;
  private JoystickButton startButton;
  private JoystickButton lStickPress;
  private JoystickButton rStickPress;

  @Override
  public void robotInit() {
    //m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);

    l1Talon = new WPI_TalonSRX(1);
    l2Talon = new WPI_TalonSRX(2);
    r1Talon = new WPI_TalonSRX(3);
    r2Talon = new WPI_TalonSRX(4);

    // aButton = new JoystickButton(0);
    // bButton = new JoystickButton(1);
    // xButton = new JoystickButton(2);
    // yButton = new JoystickButton(3);
    // lBumper = new JoystickButton(4);
    // rBumper = new JoystickButton(5);
    // backButton = new JoystickButton(6);
    // startButton = new JoystickButton(7);
    // lStickPress = new JoystickButton(8);
    // rStickPress = new JoystickButton(9);

  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    // if(m_leftStck.getRawButtonPressed){
    // }    

    double leftSpeed;
    double rightSpeed;

    leftSpeed = (m_rightStick.getY()+m_rightStick.getX())*.5;
    rightSpeed = (-m_rightStick.getY()+m_rightStick.getX())*.5;

    l1Talon.set(ControlMode.PercentOutput, leftSpeed );
    l2Talon.set(ControlMode.PercentOutput, leftSpeed );

    r1Talon.set(ControlMode.PercentOutput, rightSpeed);
    r2Talon.set(ControlMode.PercentOutput, rightSpeed);

  }

  //make the robot spin 180 degrees once
  // public void spin() {
  //   //
  //   lTalon.set(ControlMode.PercentOutput, .5);
  //   rTalon.set(ControlMode.PercentOutput, .5);
  // }
}
