/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class colorSensingWheelBot extends TimedRobot {

    // These are the states the robot can take
    public final int drive = 0;
    public final int spin = 1;
    public final int spinT = 2;
    public final int balls = 3;
    public final int extend = 4;
    public final int retract = 5;
    public final String[] states = { "DRIVE", "SPIN", "SPINT", "BALLS", "EXTEND", "RETRACT" };

    // This variable stores the robot's current state
    public int state = 0;

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

        driver = new wheelBotDriveTrain(this);

        spinner = new spinnerSystem(this);
        ballDrive = new ballSystem(this);
        lifter = new lifterSystem(this);
        vision = new visionSystem(this);
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
            s.act();
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
    }

    public void changeState(int toState) {
        resetButtons();

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
    }

    public void resetButtons() {
        for (int i = 1; i <= 10; i++)
            myController.getRawButtonPressed(i);
    }

    public void deposit() {
        if(vision.targetAcquired()){
            double xd = vision.targetXOffset();
            double yd = vision.targetYOffset();
            double targetRobotAxisAngle = Math.atan(xd / yd);
            System.out.println("targetRobotAxisAngle: "+targetRobotAxisAngle);
            double robotOffTargetAngle = vision.targetAngleOffset();
            System.out.println("robotOffTargetAngle: "+robotOffTargetAngle);
            double angleToAxis = targetRobotAxisAngle + robotOffTargetAngle;
            System.out.println("angleToAxis: "+angleToAxis);

            driver.turn(angleToAxis);
            driver.move(xd);
            driver.turn(90 * (-Math.signum(angleToAxis)));
            driver.move(yd);
            changeState(balls);
        }
    }
    @Override
    public void testPeriodic() {
        driver.operate();
        if (myController.getAButton())
            ballDrive.belt.set(ControlMode.PercentOutput, 1);
            else
                ballDrive.belt.set(ControlMode.PercentOutput, 0);  
        if (myController.getBButton())
            ballDrive.ballServo.set(95);
            else
                ballDrive.ballServo.set(0);
        if (myController.getXButton())
            lifter.extTalon.set(ControlMode.PercentOutput, .2);
            else
                lifter.extTalon.set(ControlMode.PercentOutput, 0);
        if (myController.getYButton())
            lifter.ropeTalon.set(ControlMode.PercentOutput, -.2);
            else
                lifter.ropeTalon.set(ControlMode.PercentOutput, 0);
        if (myController.getBumper(Hand.kLeft))
            spinner.spinnerMotor.setSpeed(.2);
            else
                spinner.spinnerMotor.setSpeed(0);
    }
}
