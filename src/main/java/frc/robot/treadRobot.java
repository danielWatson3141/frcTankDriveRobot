/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class treadRobot extends TimedRobot {

  private AtomicBoolean isSpinning;

  private XboxController myController;
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private TalonSRX lTalon;
  private TalonSRX rTalon;
  

  @Override
  public void robotInit() {

    myController = new XboxController(0);
    //m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);

    lTalon = new WPI_TalonSRX(1);
    rTalon = new WPI_TalonSRX(2);

  }

  @Override
  public void teleopPeriodic() {
    double leftSpeed;
    double rightSpeed;

    leftSpeed = (m_rightStick.getY()+m_rightStick.getX())*.5;
    rightSpeed = (-m_rightStick.getY()+m_rightStick.getX())*.5;

    lTalon.set(ControlMode.PercentOutput, leftSpeed );

    rTalon.set(ControlMode.PercentOutput, rightSpeed);

    if(myController.getAButtonPressed()){
      lTalon.set(ControlMode.PercentOutput, 1);
      rTalon.set(ControlMode.PercentOutput, 1);
    }
  }
}