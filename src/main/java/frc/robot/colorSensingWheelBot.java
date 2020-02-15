/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class colorSensingWheelBot extends TimedRobot {

    // These are the states the robot can take
    public final int drive = 0;
    public final int spin = 1;
    public final int spinT = 2;
    public final int balls = 3;

    public final String[] states = { "DRIVE", "SPIN", "SPINT", "BALLS", "EXTEND", "RETRACT", "FAR", "MIDDLE", "NEAR" };

    // This variable stores the robot's current state
    public int state = 0;

    // these variables define the starting point of the robot
    public int startPosition = 1;
    public final int far = 3;
    public final int middle = 2;
    public final int near = 1;

    // The Xbox controller input device
    public XboxController myController = new XboxController(0);
    public final double H = 0.1;

    public spinnerSystem spinner;
    public ballSystem ballDrive;
    public lifterSystem lifter;
    public DriveTrain driver;
    public visionSystem vision;
    public Subsystem[] systems;

    @Override
    public void robotInit() {

        myController = new XboxController(0);
        try {
            driver = new wheelBotDriveTrain(this); // set drive train type here. competition = wheelBotDriveTrain()
        } catch (Exception e) {

            e.printStackTrace();
            System.out.println("Problem occured in " + this.getClass() + ": \n" + "driver failed to activate!");
            System.out.println("driver failed to activate.");
            driver = new nullSystem.nullDriveTrain();
        }
        try {
            spinner = new spinnerSystem(this);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Problem occured in " + this.getClass() + ": \n" + "Spinner failed to activate!");
            System.out.println("spinner failed to activate.");
            spinner = new nullSystem.nullSpinnerSystem();
        }
        try {
            ballDrive = new ballSystem(this);
        } catch (Exception e) {
            e.printStackTrace();
            System.out
                    .println("Problem occured in " + this.getClass() + ": \n" + "ballDrive system failed to activate!");
            System.out.println("balldrive failed to activate.");
            ballDrive = new nullSystem.nullBallSystem();
        }
        try {
            lifter = new lifterSystem(this);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Problem occured in " + this.getClass() + ": \n" + "Lifter system failed to activate!");
            System.out.println("lifter failed to activate.");
            lifter = new nullSystem.nullLifterSystem();
        }
        try {
            vision = new visionSystem(this);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Problem occured in " + this.getClass() + ": \n" + "Vision failed to activate!");
            System.out.println("vision failed to activate.");
            vision = new nullSystem.nullVisionSystem();
        }
        systems = new Subsystem[] { driver, spinner, ballDrive, lifter, vision };
        spinner.activate();
        vision.activate();
    }

    static long currentTime;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("leftStickX", myController.getRawAxis(0));
        SmartDashboard.putNumber("leftStickY", myController.getRawAxis(1));
        SmartDashboard.putString("State", states[state]);

        currentTime = System.currentTimeMillis();

        for (Subsystem s : systems) {
            // exceptions are handled by the act
            s.act();
        }

    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
        for (Subsystem s : systems) {
            s.activate();
        }
        // code of autonomous period
        // three cases for three starting positions
        switch (startPosition) {
        case far:
            driver.move(1.4859);
        case middle:
            driver.move(1.4859);
            driver.turn(-90);
        case near:
            deposit();
            driver.move(-3.2);
            break;
        }

    }

    @Override
    public void teleopInit() {
        super.teleopInit();
        for (Subsystem s : systems) {
            // activates everything
            s.activate();
        }
    }

    // This method is called about 10 times per second while the robot is set to
    // teleop mode.
    @Override
    public void teleopPeriodic() {

        if (myController.getBButtonPressed()) {
            changeState(drive);
        }

        // This section controls state behavior. It defines state transitions and
        // initializations
        switch (state) {

        case drive:

            if (myController.getYButtonPressed()) {
                if(myController.getBumper(Hand.kLeft)){
                    changeState(spinT);
                } else {
                    changeState(spin);    
                }
            } 
            else if (myController.getXButtonPressed()) {
                changeState(balls);
                
            }

            break;
        case spin:
            spinner.spin();

            if (spinner.sectorCount == 28) {
                changeState(drive);
            }

            break;
        case spinT:
            spinner.spinT();
            if (spinner.targetColor == spinner.currentColor) {
                changeState(drive);
            }
            break;
        }
    }

    private void changeState(int toState) {
        try {
            resetButtons();
            System.out.println("Changing state: " + state + " -> " + toState);
            switch (state) {
            case spin:
                spinner.stopSpinning();
                break;
            case spinT:
                spinner.stopSpinning();
                break;
            case balls:
                ballDrive.closeGate();
                break;
            }

            switch (toState) {
            case spin:
                spinner.sectorCount = 0;
                spinner.currentColor = spinner.dColor;
                break;
            case spinT:
                spinner.currentColor = spinner.dColor;
                break;
            case balls:
                ballDrive.openGate();
                break;
            case drive:
                break;
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error in switching states! ");
        } finally {
            state = toState;
        }

    }

    public void resetButtons() {
        for (int i = 1; i <= 10; i++)
            myController.getRawButtonPressed(i);
    }

    public void deposit() {
        // if the vision target is aquired, the robot will move in the x direction
        // turn 90 deg and move in the y direction
        // then it will go into the balls state and release the balls
        try {
            if (vision.targetAcquired()) {

                double xd = vision.targetXOffset();
                double yd = vision.targetYOffset();
                double targetRobotAxisAngle = Math.atan(xd / yd);
                System.out.println("targetRobotAxisAngle: " + targetRobotAxisAngle);
                double robotOffTargetAngle = vision.targetAngleOffset();
                System.out.println("robotOffTargetAngle: " + robotOffTargetAngle);
                double angleToAxis = targetRobotAxisAngle + robotOffTargetAngle;
                System.out.println("angleToAxis: " + angleToAxis);

                driver.turn(angleToAxis);
                driver.move(xd);
                driver.turn(90 * (-Math.signum(angleToAxis)));
                driver.move(yd);
                changeState(balls);

            } else {
                System.out.println("Failed to acquire target!!!");
            }
        } catch (Exception e) {
            System.out.println("Problem encountered while depositing! ");
            e.printStackTrace();
        }
    }

    double servoAngle = .5;

    // tester code
    @Override
    public void testPeriodic() {

        // operate the drivetrain (take controller input)

        driver.operate();

        // each button is associated with a motor
        if (myController.getAButton())
            ballDrive.belt.set(ControlMode.PercentOutput, 1);
        else
            ballDrive.belt.set(ControlMode.PercentOutput, 0);
        if (myController.getBButton())
            ballDrive.openGate();
        else
            ballDrive.closeGate();

        if (myController.getXButton())
            lifter.extend();
        else
            lifter.retract();
        if (myController.getYButton())
            lifter.ropeTalon.set(ControlMode.PercentOutput, -.2);
        else
            lifter.ropeTalon.set(ControlMode.PercentOutput, 0);
        if (myController.getBumper(Hand.kLeft))
            spinner.spinnerMotor.set(ControlMode.PercentOutput, .2);
        else
            spinner.spinnerMotor.set(ControlMode.PercentOutput, 0);
        if (myController.getBumper(Hand.kRight))
            // moves 1 m
            driver.move(1);
        if (myController.getStickButton(Hand.kRight))
            // turn 90 deg
            driver.turn(90);
    }
}
