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
public class Robot extends TimedRobot {
  private GenericHID myController;
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private TalonSRX lTalon;
  private TalonSRX rTalon;
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

    myController = new XboxController(0);
    //m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);

    lTalon = new WPI_TalonSRX(1);
    rTalon = new WPI_TalonSRX(2);

    aButton = new JoystickButton(myController,0); //servo button

    spinOnce ss = new spinOnce(this);

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

    double leftSpeed;
    double rightSpeed;

    leftSpeed = (m_rightStick.getY()+m_rightStick.getX())*.5;
    rightSpeed = (-m_rightStick.getY()+m_rightStick.getX())*.5;

    lTalon.set(ControlMode.PercentOutput, leftSpeed );

    rTalon.set(ControlMode.PercentOutput, rightSpeed);
    
  }

  private class spinOnce implements Runnable{
    
    Robot me;

    public spinOnce(Robot mi){
        me = mi;
    }

    public void run(){
        me.lTalon.set(ControlMode.PercentOutput, .5 );

        me.rTalon.set(ControlMode.PercentOutput, .5 );
    }
  }

}
