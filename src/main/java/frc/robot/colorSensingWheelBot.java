/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.GenericHID.Hand;



public class colorSensingWheelBot extends TimedRobot {

    // possible ICU code
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private static final String kYawDefault = "Z-Axis";
    private static final String kYawXAxis = "X-Axis";
    private static final String kYawYAxis = "Y-Axis";
    private String m_yawSelected;
    private ADIS16470_IMU.IMUAxis m_yawActiveAxis;
    private final SendableChooser<String> m_yawChooser = new SendableChooser<>();
    private boolean m_runCal = false;
    private boolean m_configCal = false;
    private boolean m_reset = false;
    private boolean m_setYawAxis = false;
    private final ADIS16470_IMU m_imu = new ADIS16470_IMU();

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

    private final int RED = 0;
    private final int YELLOW = 0;
    private final int BLUE = 0;
    private final int GREEN = 0;



    // an array for converting int to string and back
    private final String[] colors = { red, yellow, blue, green };

    // The color that the user has designated the target color
    private int targetColor = 0;

    // The color detected by the sensor
    private int dColor;
    private int prevDColor;

    // These are the two motors characteristic of a treadbot
    private TalonSRX l1Talon;
    private TalonSRX r1Talon;
    private TalonSRX l2Talon;
    private TalonSRX r2Talon;

    // This is the motor for spinning the spinner wheel
    private PWM spinnerMotor;

    // This is the motor for the belt and the servo motor for the ball mechanism
    private TalonSRX chain;
    private Servo ballServo;

    // This is the motors and hall effect sensors for the lifter mechanisms
    private Talon extTalon;
    private Talon ropeTalon;
    private DigitalInput heffectTop;
    private DigitalInput heffectBottom;

    // This is network table data for the limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //assumed speed robot does things at maximum rate
    private final double degreesPerSecond = 270; //max turn rate in deg/s
    private final double metersPerSecond = 8; //max drive speed in m/s
    private final double H = 0.1;

    private long timeToQuit;
    private long currentTime;

    Encoder enc;

    // count per revolution for Encoder
    private static final double cpr = 360;

    //diameter of wheels
    private static final double whd = .1524;

    //half distance between the wheels
    private static final double wheelRadius = .381;

    public static final ADIS16470_IMU imu = new ADIS16470_IMU();


    @Override
    public void robotInit() {

        myController = new XboxController(0);
        // m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
        //leftStick = new Joystick(1);

        l1Talon = new TalonSRX(3);
        l2Talon = new TalonSRX(4);

        r1Talon = new TalonSRX(3);
        r2Talon = new TalonSRX(4);

        spinnerMotor = new PWM(1);
        chain = new TalonSRX(5);


        extTalon = new Talon(5);
        ropeTalon = new Talon(3);
        heffectTop = new DigitalInput(0);
        heffectBottom = new DigitalInput(1);

        ballServo = new Servo(2);

        m_yawChooser.setDefaultOption("Z-Axis", kYawDefault);
        m_yawChooser.addOption("X-Axis", kYawXAxis);
        m_yawChooser.addOption("Y-Axis", kYawYAxis);
        SmartDashboard.putData("IMUYawAxis", m_yawChooser);
        SmartDashboard.putBoolean("RunCal", false);
        SmartDashboard.putBoolean("ConfigCal", false);
        SmartDashboard.putBoolean("Reset", false);
        SmartDashboard.putBoolean("SetYawAxis", false);

        //enc = new Encoder(0, 1);


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
        
        currentTime = System.currentTimeMillis();

        // Provides smartboard data for the limelight
        double xPos = tx.getDouble(0.0);
        double yPos = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", xPos);
        SmartDashboard.putNumber("LimelightY", yPos);
        SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("YawAngle", m_imu.getAngle());
        SmartDashboard.putNumber("XCompAngle", m_imu.getXComplementaryAngle());
        SmartDashboard.putNumber("YCompAngle", m_imu.getYComplementaryAngle());
        m_runCal = SmartDashboard.getBoolean("RunCal", false);
        m_configCal = SmartDashboard.getBoolean("ConfigCal", false);
        m_reset = SmartDashboard.getBoolean("Reset", false);
        m_setYawAxis = SmartDashboard.getBoolean("SetYawAxis", false);
        m_yawSelected = m_yawChooser.getSelected();
    
        // Set IMU settings
        if (m_configCal) {
          m_imu.configCalTime(ADIS16470_IMU.ADIS16470CalibrationTime._8s);
          m_configCal = SmartDashboard.putBoolean("ConfigCal", false);
        }
        if (m_reset) {
          m_imu.reset();
          m_reset = SmartDashboard.putBoolean("Reset", false);
        }
        if (m_runCal) {
          m_imu.calibrate();
          m_runCal = SmartDashboard.putBoolean("RunCal", false);
        }
        
        // Read the desired yaw axis from the dashboard
        if (m_yawSelected == "X-Axis") {
          m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kX;
        }
        else if (m_yawSelected == "Y-Axis") {
          m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kY;
        }
        else {
          m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kZ;
        }
        // Set the desired yaw axis from the dashboard
        if (m_setYawAxis) {
          m_imu.setYawAxis(m_yawActiveAxis);
          m_setYawAxis = SmartDashboard.putBoolean("SetYawAxis", false);
        }
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
                //extTalon.set( 0);
            } else {
                //extTalon.set(-.2);
            }

            if (myController.getYButtonPressed()) {
                changeState(spin);
                break;
            } else if (myController.getBumperPressed(Hand.kLeft)) {
                changeState(spinT);;
                break;
            } else if (myController.getXButtonPressed()) {
                changeState(balls);
                break;
            } else if (myController.getStickButtonPressed(Hand.kRight)) {
                changeState(extend);;
                break;
            }

            break;
        case spin:
            spin();
            if (myController.getBButtonPressed()) {
                changeState(drive);
            }

            break;
        case spinT:
            spinT();
            if (targetColor == currentColor) {
                changeState(drive);
            } else if (myController.getBButtonPressed()) {
                changeState(drive);
            }
            break;

        case balls:
            balls();
            drive();
            if (myController.getBButtonPressed()) {
                changeState(drive);
            }
            break;

        case extend:
            drive();
            if (!heffectTop.get()) {
                extTalon.set(0);
            } else {
                extTalon.set(.2);
            }

            if (myController.getBButtonPressed()) {
                changeState(drive);
            }

            if(myController.getStickButtonPressed(Hand.kRight)){
                changeState(retract);
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
                changeState(drive);
            }
            break;
        }
    }

    private void changeState(int toState){
        resetButtons();

        switch(toState){
            case spin:
                sectorCount = 0;
                currentColor = dColor;
                break;
            case spinT:
                currentColor = dColor;
                break;
            case balls:
                break;
            case drive:
                ballServo.setAngle(0);
                chain.set(ControlMode.PercentOutput, 0);
                spinnerMotor.setSpeed(0);
                break;
        }
        state = toState;
    }

    private void resetButtons(){
        for(int i=1; i<=10; i++)
            myController.getRawButtonPressed(i);
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
        if (g - b > .25) {
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

        leftSpeed = (myController.getRawAxis(1) + myController.getRawAxis(0)) * .5;
        rightSpeed = (-myController.getRawAxis(1) + myController.getRawAxis(0)) * .5;

        setWheelSpeed(leftSpeed, rightSpeed);

        if (myController.getAButton()) {
            chain.set(ControlMode.PercentOutput, 1);
        }
    }

    private void spin() {
        spinnerMotor.setSpeed(.30);
        if (dColor == (currentColor + 1) % 4) {
            sectorCount++;
            SmartDashboard.putNumber("sectorCount", sectorCount);
            currentColor++;
            currentColor %= 4;
        }
        if (sectorCount == 28) {
            state = drive;
            spinnerMotor.setSpeed(0);
        }

    }

    private void spinT() {
        spinnerMotor.setSpeed(.25);
        if (dColor == (currentColor + 1) % 4) {
            SmartDashboard.putString("currentColor", colors[currentColor]);
            currentColor++;
            currentColor %= 4;
        }
    }

    private void balls() {
        ballServo.setAngle(135);
        chain.set(ControlMode.PercentOutput, 1);
    }

    private void deposit(double distance, double theta) {
        double xd = distance*Math.cos(theta);
        double yd = distance*Math.sin(theta) - H;
        turn(theta);
        move(xd);
        turn(90*(-Math.signum(theta)));
        move(yd);
    }

    //theta: degrees
    //turn at half speed
    private void turn(double theta){

        double totalDistance = 0;
        double leftSpeed, rightSpeed;
        distanceTravelled(Hand.kLeft);

        leftSpeed = rightSpeed = Math.signum(theta) * .5;
        setWheelSpeed(leftSpeed, rightSpeed);

        double distance = theta * wheelRadius * Math.PI / 180;

        while(totalDistance < distance){
            totalDistance += distanceTravelled(Hand.kLeft);
        }

        leftSpeed = rightSpeed = .0;
        setWheelSpeed(leftSpeed, rightSpeed);
    }

    //distance: meters
    //move at half speed
    private void move(double distance){
        double totalDistance = 0;
        double leftSpeed, rightSpeed;
        distanceTravelled(Hand.kLeft);
        leftSpeed = .5 * Math.signum(distance);
        rightSpeed = -.5 * Math.signum(distance);
        setWheelSpeed(leftSpeed, rightSpeed);

        while(totalDistance < distance){
            totalDistance += distanceTravelled(Hand.kLeft);
        }

        leftSpeed = rightSpeed = .0;
        setWheelSpeed(leftSpeed, rightSpeed);
    }

    void setWheelSpeed(double leftSpeed, double rightSpeed){
        l1Talon.set(ControlMode.PercentOutput, leftSpeed);
        l2Talon.set(ControlMode.PercentOutput, leftSpeed);
        r1Talon.set(ControlMode.PercentOutput, rightSpeed);
        r2Talon.set(ControlMode.PercentOutput, rightSpeed);
    }

    //returns distance travelled since last call
    double distanceTravelled(Hand side){
        return 0;
    }
}
