/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class colorSensingWheelBot extends TimedRobot {

    // These are the states the robot can take
    private final int drive = 0;
    private final int spin = 1;
    private final int spinT = 2;
    private final int balls = 3;
    private final int extend = 4;
    private final int retract = 5;
    private final String[] states = { "DRIVE", "SPIN", "SPINT", "BALLS", "EXTEND", "RETRACT" };

    // This variable stores the robot's current state
    public int state = 0;

    // The Xbox controller input device
    public XboxController myController = new XboxController(0);
    private final double H = 0.1;

    private spinnerSystem spinner;
    private ballSystem ballDrive;
    private lifterSystem lifter;
    private DriveTrain driver;
    private visionSystem vision;
    private Subsystem[] systems;

    @Override
    public void robotInit() {

        try {
            myController = new XboxController(0);
        } catch (Exception e) {
            System.out.println("Controller failed to init! " + e.getStackTrace());
        }
        try {
            driver = new wheelBotDriveTrain(this);
        } catch (Exception e) {
            System.out.println("drivetrain failed to init! " + e.getStackTrace());
        }
        try {
            spinner = new spinnerSystem(this);
        } catch (Exception e) {
            System.out.println("spinner failed to init! " + e.getStackTrace());
        }
        try {
            ballDrive = new ballSystem(this);
        } catch (Exception e) {
            System.out.println("ballsystem failed to init! " + e.getStackTrace());
        }
        try {
            lifter = new lifterSystem(this);
        } catch (Exception e) {
            System.out.println("lifter failed to init! " + e.getStackTrace());
        }
        try {
            vision = new visionSystem(this);
        } catch (Exception e) {
            System.out.println("vision failed to init! " + e.getStackTrace());
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
            try {
                s.act();
            } catch (Exception e) {
                System.out.println(
                        "Subsystem " + s.getClass().toString() + " has encountered a problem! " + e.getStackTrace());
            }
        }

    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
        for (Subsystem s : systems) {
            s.activate();
        }
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
        for (Subsystem s : systems) {
            s.activate();
        }
    }

    // This method is called about 10 times per second while the robot is set to
    // teleop mode.
    @Override
    public void teleopPeriodic() {

        try {

            if (myController.getBButtonPressed()) {
                changeState(drive);
            }

            // This section controls state behavior. It defines state transitions and
            // initializations
            switch (state) {

            case drive:

                if (myController.getYButtonPressed()) {
                    changeState(spin);
                    break;
                } else if (myController.getBumperPressed(Hand.kLeft)) {
                    changeState(spinT);
                    break;
                } else if (myController.getXButtonPressed()) {
                    changeState(balls);
                    break;
                } else if (myController.getStickButtonPressed(Hand.kRight)) {
                    changeState(extend);
                    break;
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

            case balls:
                // action takes place in state change
                break;

            case extend:
                lifter.extend();
                if (myController.getStickButtonPressed(Hand.kRight)) {
                    changeState(retract);
                }

                break;

            case retract:
                lifter.retract();
                break;
            }
        } catch (Exception e) {
            System.out.println("State machine has encountered a problem! " + e.getStackTrace());
        }
    }

    private void changeState(int toState) {
        try {
            resetButtons();
            System.out.println("Changing state: "+state+" -> "+toState);
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
            state = toState;
        } catch (Exception e) {
            System.out.println("problem changing states! " + e.getStackTrace());
        }
    }

    private void resetButtons() {
        for (int i = 1; i <= 10; i++)
            myController.getRawButtonPressed(i);
    }

    private void deposit() {
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
            }
        } catch (Exception e) {
            System.out.println("Problem depositing! " + e.getStackTrace());
        }
    }

}
