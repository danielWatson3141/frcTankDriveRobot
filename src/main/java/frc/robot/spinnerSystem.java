package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class spinnerSystem extends Subsystem {

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    public final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
     * The device will be automatically initialized with default parameters.
     */
    public ColorSensorV3 m_colorSensor;

    // The color currently detected by the sensor
    Color detectedColor;

    // Intensity of IR radiation hitting the sensor. Can detect heat and light
    double IR;

    // Intensity of transmitted IR light returning to sensor. High number means
    // close subject
    double proximity;

    // These variables store the number of sectors seen so far on the color wheel
    public int sectorCount = 0;

    // This stores the color we saw most recently not counting backtracks
    public int currentColor = 0;

    // short forms for color strings
    public final String red = "RED";
    public final String yellow = "YELLOW";
    public final String blue = "BLUE";
    public final String green = "GREEN";

    // an array for converting int to string and back
    public final String[] colors = { red, yellow, blue, green };

    // The color that the user has designated the target color
    public int targetColor = 0;

    // The color detected by the sensor
    int dColor = 0;

    // This is the motor for spinning the spinner wheel
    public PWM spinnerMotor;

    public spinnerSystem(){}

    public spinnerSystem(colorSensingWheelBot theRobot) {
        super(theRobot);
        spinnerMotor = new PWM(5);
        m_colorSensor = new ColorSensorV3(i2cPort);
    }

    @Override
    public void operate() {
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
        SmartDashboard.putString("Color Sensed", ColorToString(detectedColor));
        SmartDashboard.putString("Target Color", colors[targetColor]);

        // this allows the driver to switch the target color
        if (controller.getBumperPressed(Hand.kRight)) {
            targetColor += 1;
            if (targetColor > 3) {
                targetColor = 0;
            }
        }

        //TODO: define constants for states
        if(robot.state == 1){
            spin();
        }

        if(robot.state == 2){
            spinT();
        }
    }

    public void spin() {
        //System.out.println("spinning");
        //spinnerMotor.set(ControlMode.PercentOutput, .2);
        spinnerMotor.setSpeed(.20);
        if (dColor == (currentColor + 1) % 4) {
            sectorCount++;
            SmartDashboard.putNumber("sectorCount", sectorCount);
            currentColor++;
            currentColor %= 4;
        }
    }

    public void spinT() {
        //System.out.println("spinning");
       // spinnerMotor.set(ControlMode.PercentOutput, .15);
        spinnerMotor.setSpeed(.15);
        if (dColor == (currentColor + 1) % 4) {
            SmartDashboard.putString("currentColor", colors[currentColor]);
            currentColor++;
            currentColor %= 4;
        }
    }

    public void stopSpinning() {
        //spinnerMotor.set(ControlMode.PercentOutput, 0);
        spinnerMotor.setSpeed(0);
    }

    public int ColorToInt(Color c) {
        double r = c.red;
        double g = c.green;
        double b = c.blue;

        // This code returns a number which corresponds to one of the four colors
        // The numbers are determined based on the RGB values that the color senors
        // detects
        // The equations were constructed based off of testing the RGB values of each of
        // the four colors
        if (r > g * .8 && r > b)
            return 0;
        if (g - b > .2) {
            if (r > b && g > b)
                return 1;
            return 3;
        }
        return 2;

    }

    public String ColorToString(Color c) {
        return colors[ColorToInt(c)];
    }
}