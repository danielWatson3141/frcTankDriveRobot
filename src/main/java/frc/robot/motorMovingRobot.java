/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import java.util.concurrent.atomic.AtomicBoolean; 

import java.lang.Runnable;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.Talon;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class motorMovingRobot extends TimedRobot {

  private AtomicBoolean isSpinning;

  private XboxController myController;
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private TalonSRX l1Talon;
  private TalonSRX rTalon;
  private TalonSRX l2Talon;
  private VictorSPX rVEX;
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
  private Talon motorcontrol;

  @Override
  public void robotInit() {

    System.out.println("Initializing");

    isSpinning = new AtomicBoolean(false);

    myController = new XboxController(0);
    //m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(1);


    l1Talon = new WPI_TalonSRX(1);
    rVEX = new WPI_VictorSPX(0);
    l2Talon = new WPI_TalonSRX(3);
    rTalon = new WPI_TalonSRX(4);

    motorcontrol = new Talon(2);

  
     
    //aButton = myController.getAButton(); //servo button

    //myController.
    //spinOnce ss = new spinOnce(this);

    //aButton.whenPressed(new InstantCommand(ss));


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

    //if(! isSpinning.get()){
        double leftSpeed;
        double rightSpeed;

        leftSpeed = (m_leftStick.getY()+m_leftStick.getX())*.5;
        rightSpeed = (-m_leftStick.getY()+m_leftStick.getX())*.5;

        rTalon.set(ControlMode.PercentOutput, rightSpeed);
        rVEX.set(ControlMode.PercentOutput, rightSpeed);
        l1Talon.set(ControlMode.PercentOutput, leftSpeed);
        l2Talon.set(ControlMode.PercentOutput, leftSpeed);

        motorcontrol.set(leftSpeed);
   // }
    
  }

  // private class spinOnce implements Runnable{
    
  //   Robot me;

  //   public spinOnce(Robot mi){
  //       me = mi;
  //   }

  //   public void run(){
  //       System.out.println("Button Pressed");
  //       me.isSpinning.set(true);

  //       me.lTalon.set(ControlMode.PercentOutput, .5 );
  //       me.rTalon.set(ControlMode.PercentOutput, .5 );
  //       try{Thread.sleep(1000);}catch(Exception e){};
  //       me.isSpinning.set(false);
  //   }
  // }

}
