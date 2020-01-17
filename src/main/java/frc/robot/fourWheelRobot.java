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




import java.lang.Runnable;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.*;

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
  private Servo servo;

  @Override
  public void robotInit() {

    myController = new XboxController(0);
    //m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);

    l1Talon = new WPI_TalonSRX(1);
    l2Talon = new WPI_TalonSRX(2);
    r1Talon = new WPI_TalonSRX(3);
    r2Talon = new WPI_TalonSRX(4);

    aButton = new JoystickButton(myController,0); //servo button
    servo = new Servo(0);

    ServoSpin ss = new ServoSpin(servo);

    aButton.whenPressed(new InstantCommand(ss));


    bButton = new JoystickButton(myController,1);
    xButton = new JoystickButton(myController,2);
    yButton = new JoystickButton(myController,3);
    lBumper = new JoystickButton(myController,4);
    rBumper = new JoystickButton(myController,5);
    backButton = new JoystickButton(myController,6);
    startButton = new JoystickButton(myController,7);
    lStickPress = new JoystickButton(myController,8);
    rStickPress = new JoystickButton(myController,9);

    

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

  private class ServoSpin implements Runnable{
    Servo servo;
    public ServoSpin(Servo s){
      servo = s;
    }

    public void run(){
      double angle = servo.getAngle();
      servo.setAngle(angle + 45);
    }
  }

}
