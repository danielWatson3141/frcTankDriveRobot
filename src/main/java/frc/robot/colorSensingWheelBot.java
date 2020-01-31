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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class colorSensingWheelBot extends TimedRobot {
    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
     * The device will be automatically initialized with default parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    // These are the states the robot can take
    private final int drive = 0;
    private final int spin = 1;
    private final int spinT = 2;
    private final int balls = 3;
    private final int extend = 4;
    private final int retract = 5;
    private final String[] states = { "DRIVE", "SPIN", "SPINT", "BALLS", "EXTEND", "RETRACT" };

    // This variable stores the robot's current state
    private int state = 0;

    // The Xbox controller input device
    XboxController myController = new XboxController(0);

    // The color currently detected by the sensor
    Color detectedColor;

    // Intensity of IR radiation hitting the sensor. Can detect heat and light
    double IR;

    // Intensity of transmitted IR light returning to sensor. High number means
    // close subject
    double proximity;

    // These variables store the number of sectors seen so far on the color wheel
    private int sectorCount = 0;

    // This stores the color we saw most recently not counting backtracks
    private int currentColor;

    // short forms for color strings
    private final String red = "RED";
    private final String yellow = "YELLOW";
    private final String blue = "BLUE";
    private final String green = "GREEN";

    // an array for converting int to string and back
    private final String[] colors = { red, yellow, blue, green };

    // The color that the user has designated the target color
    private int targetColor = 0;

    private int dColor;

    

    // These are the two motors characteristic of a treadbot
    private TalonSRX l1Talon;
    private TalonSRX r1Talon;
    private TalonSRX l2Talon;
    private TalonSRX r2Talon;

    // This is the motor for spinning the spinner wheel
    private TalonSRX spinnerMotor;

    // This is the motor for the belt and the servo motor for the ball mechanism
    private TalonSRX chain;
    private Servo ballServo;

    // This is the motors and hall effect sensors for the lifter mechanisms
    private Talon extTalon;
    private Talon ropeTalon;
    private DigitalInput heffectTop;
    private DigitalInput heffectBottom;

    //assumed speed robot does things at maximum rate
    private double degreesPerSecond; //max turn rate in deg/s
    private double metersPerSecond; //max drive speed in m/s

    @Override
    public void robotInit() {

        myController = new XboxController(0);
        // m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
        //leftStick = new Joystick(1);

        l1Talon = new TalonSRX(3);
        l2Talon = new TalonSRX(4);

        r1Talon = new TalonSRX(1);
        r2Talon = new TalonSRX(2);

        spinnerMotor = new TalonSRX(8);
        chain = new TalonSRX(5);

        extTalon = new Talon(1);
        ropeTalon = new Talon(3);
        heffectTop = new DigitalInput(0);
        heffectBottom = new DigitalInput(1);

        ballServo = new Servo(2);

    }

    @Override
    public void robotPeriodic() {

        /**
         * The method GetColor() returns a normalized color value from the sensor and
         * can be useful if outputting the color to an RGB LED or similar. To read the
         * raw color, use GetRawColor().
         * 
         * The color sensor works best when within a few inches from an object in well
         * lit conditions (the built in LED is a big help here!). The farther an object
         * is the more light from the surroundings will bleed into the measurements and
         * make it difficult to accurately determine its color.
         */
        detectedColor = m_colorSensor.getColor();

        // convert the detected color to an int
        dColor = ColorToInt(detectedColor);

        // show the detected color on the dashboard
        SmartDashboard.putString("detected Color", colors[dColor]);

        SmartDashboard.putString("State", states[state]);

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        IR = m_colorSensor.getIR();

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putString("Color Sensed", ColorToString(detectedColor));
        SmartDashboard.putString("Target Color", colors[targetColor]);

        /**
         * In addition to RGB IR values, the color sensor can also return an infrared
         * proximity value. The chip contains an IR led which will emit IR pulses and
         * measure the intensity of the return. When an object is close the value of the
         * proximity will be large (max 2047 with default settings) and will approach
         * zero when the object is far away.
         * 
         * Proximity can be used to roughly approximate the distance of an object or
         * provide a threshold for when an object is close enough to provide accurate
         * color values.
         */
        proximity = m_colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);

        SmartDashboard.putNumber("leftStickX", myController.getRawAxis(0));
        SmartDashboard.putNumber("leftStickY",  myController.getRawAxis(1));
        
    }

    // This method is called about 10 times per second while the robot is set to
    // teleop mode.
    @Override
    public void teleopPeriodic() {

        // This converts the color to an integer representing one of the four colors in
        // play
        if (myController.getBumperPressed(Hand.kRight)) {
            targetColor += 1;
            if (targetColor > 3) {
                targetColor = 0;
            }
        }

        // This section controls state behavior. It defines state transitions and
        // initializations
        switch (state) {

        case drive:
            drive();

            if (!heffectBottom.get()) {
                extTalon.set( 0);
            } else {
                extTalon.set(-.2);
            }

            if (myController.getYButtonPressed()) {
                state = spin;
                sectorCount = 0;
                currentColor = dColor;
                break;
            } else if (myController.getBumperPressed(Hand.kLeft)) {
                state = spinT;
                break;
            } else if (myController.getXButtonPressed()) {
                state = balls;
                break;
            } else if (myController.getStickButtonPressed(Hand.kRight)) {
                state = extend;
                break;
            }

            break;
        case spin:
            spin();
            if (myController.getBButtonPressed()) {
                spinnerMotor.set(ControlMode.PercentOutput, 0);
                state = drive;
            }

            break;
        case spinT:
            spinT();
            if (targetColor == dColor) {
                spinnerMotor.set(ControlMode.PercentOutput, 0);
                state = drive;
            } else if (myController.getBButtonPressed()) {
                spinnerMotor.set(ControlMode.PercentOutput, 0);
                state = drive;
            }
            break;

        case balls:
            balls();
            if (myController.getBButtonPressed()) {
                ballServo.setAngle(0);
                chain.set(ControlMode.PercentOutput, 0);
                state = drive;
            }
            break;

        case extend:
            drive();
            if (!heffectTop.get()) {
                extTalon.set(0);
                //state = drive;
            } else {
                extTalon.set(.2);
            }

            if (myController.getBButtonPressed()) {
                state = drive;
            }

            if(myController.getStickButtonPressed(Hand.kRight)){
                state = retract;
            }

            break;

        case retract:
            
            ropeTalon.set(myController.getRawAxis(1));
            if (!heffectBottom.get()) {
                extTalon.set(0);
            }
            else {
                extTalon.set( -.2);
            if (myController.getBButtonPressed())
                state = drive;
            }
            break;
        }
    }

    private int ColorToInt(Color c) {
        double r = c.red;
        double g = c.green;
        double b = c.blue;



        // This code returns a number which corresponds to one of the four colors
        // The numbers are determined based on the RGB values that the color senors
        // detects
        // The equations were constructed based off of testing the RGB values of each of
        // the four colors
        if (r > g * .8  && r > b)
            return 0;
        if (g - b > .2) {
            if (r > b && g > b)
                return 1;
            return 3;
        }
        return 2;

    }

    private String ColorToString(Color c) {
        return colors[ColorToInt(c)];
    }

    private void drive() {
        double leftSpeed;
        double rightSpeed;

        leftSpeed = (myController.getRawAxis(1) + myController.getRawAxis(0)) * .0;
        rightSpeed = (-myController.getRawAxis(1) + myController.getRawAxis(0)) * .0;

        l1Talon.set(ControlMode.PercentOutput, leftSpeed);
        l2Talon.set(ControlMode.PercentOutput, leftSpeed);

        r1Talon.set(ControlMode.PercentOutput, rightSpeed);
        r2Talon.set(ControlMode.PercentOutput, rightSpeed);

        if (myController.getAButton()) {
            chain.set(ControlMode.PercentOutput, 1);
        }
    }

    private void spin() {
        spinnerMotor.set(ControlMode.PercentOutput, .1);
        if (dColor == (currentColor + 1) % 4) {
            sectorCount++;
            currentColor++;
            currentColor %= 4;
        }
        if (sectorCount == 28) {
            state = drive;
        }

    }

    private void spinT() {
        spinnerMotor.set(ControlMode.PercentOutput, .1);
    }

    private void balls() {
        ballServo.setAngle(135);
        chain.set(ControlMode.PercentOutput, 1);
    }

    private void deposit() {

    }

    //theta: degrees
    private void turn(double theta){

    }

    //distance: meters
    private void move(double distance){

    }


}