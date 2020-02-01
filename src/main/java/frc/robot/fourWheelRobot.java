/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class fourWheelRobot extends TimedRobot {
  private GenericHID myController;
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private TalonSRX l1Talon;
  private TalonSRX r1Talon;
  private TalonSRX l2Talon;
  private TalonSRX r2Talon;

  private long timeToQuit;
  private long currentTime;

  @Override
  public void robotInit() {

    myController = new XboxController(0);
    // m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    // m_rightStick = new Joystick(1);

    l1Talon = new TalonSRX(3);
    l2Talon = new TalonSRX(4);

    r1Talon = new TalonSRX(1);
    r2Talon = new TalonSRX(2);
  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub

    timeToQuit = currentTime + 5000;

    super.teleopInit();
  }

  @Override
  public void robotPeriodic() {
    currentTime = System.currentTimeMillis();
  }

  @Override
  public void teleopPeriodic() {
    // m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    // if(m_leftStck.getRawButtonPressed){
    // }

    double leftSpeed;
    double rightSpeed;

    if(currentTime > timeToQuit)
      leftSpeed = rightSpeed = 0;
    else
      leftSpeed = rightSpeed = .5;
    
    l1Talon.set(ControlMode.PercentOutput, leftSpeed);
    l2Talon.set(ControlMode.PercentOutput, leftSpeed);

    r1Talon.set(ControlMode.PercentOutput, rightSpeed);
    r2Talon.set(ControlMode.PercentOutput, rightSpeed);

  }

}
