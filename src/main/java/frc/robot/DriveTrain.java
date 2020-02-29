package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DriveTrain extends Subsystem {

    double laccumulator = 0;
    double raccumulator = 0;
    long previousTime = 0; // ms since Jan 1 1970

    private Spark ledDriver;
    AnalogInput mic;
    


    static final double gearRatio = 1 / 6.0; // unitless
    static final double wheelRadius = .15; // meters

    static final double unitsPerRotation = 360;
    static final double unitsPerRadian = unitsPerRotation / (2 * Math.PI);

    static final Double STEERING_STRENGTH = .5;
    double color = .85;
    private ADIS16470_IMU imu;
    private double angle;
    

    public DriveTrain() {

    }

    public DriveTrain(colorSensingWheelBot theRobot) {
        super(theRobot);
        ledDriver = new Spark(6);
        imu = new ADIS16470_IMU(); 
        mic = new AnalogInput(0);
        mic.setOversampleBits(4);
        mic.setAverageBits(4);
        
    }

    public void operate() {
        angle = imu.getAngle();
        SmartDashboard.putNumber("Angle", angle);

        double soundVolume = mic.getAverageValue();
        SmartDashboard.putNumber("Mic value", soundVolume);

        ledDriver.set(soundVolume);

        double leftSpeed;
        double rightSpeed;

        double xAxis = -controller.getRawAxis(1); // dampen the x axis to improve handling
        double yAxis = controller.getRawAxis(0)* STEERING_STRENGTH;

        leftSpeed = (xAxis + yAxis) * .5;
        rightSpeed = (-xAxis + yAxis) * .5;

        setWheelSpeed(leftSpeed, rightSpeed);
        accumulate();
    }

    // theta: degrees
    // turn at half speed
    public void turn(double theta) {
        try {
            double initialAngle = imu.getAngle();
            double leftSpeed, rightSpeed;

            leftSpeed = rightSpeed = -Math.signum(theta) * .2;
            setWheelSpeed(leftSpeed, rightSpeed);

            // .5 is because we have 2 "wheels" turning opposite directions

            while (true) {
                angle = imu.getAngle();
                System.out.println(angle);
                if(controller.getBButtonPressed()){
                    return;
                }
                accumulate();

                if( theta > 0 && angle > initialAngle + theta*.9)
                    break;
                if( theta < 0 && angle < initialAngle + theta*.9)
                    break;
                
            }

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Problem turning! ");
        } finally {
            setWheelSpeed(0, 0); // this always executes
        }
    }

    // distance: meters
    // move at half speed
    public void move(double distance) {
        try {
            resetAccumulator();
            double leftSpeed, rightSpeed;
            distanceTravelled(Hand.kLeft);
            leftSpeed = .1 * Math.signum(distance);
            rightSpeed = -.1 * Math.signum(distance);
            setWheelSpeed(leftSpeed, rightSpeed);
            double distanceSinceStart = 0;

            while (distanceSinceStart / 1000 <= distance*4.5) {
                distanceSinceStart -= distanceTravelled(Hand.kLeft);
                System.out.println("L:"+rotationRate(Hand.kLeft)+" R:"+rotationRate(Hand.kRight));
                if(controller.getBButtonPressed()){
                    setWheelSpeed(0, 0);
                    return;
                }
                accumulate();
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Problem turning! ");
        } finally {
            setWheelSpeed(0, 0); // this always executes
        }
    }

    private double distanceTravelled(Hand side) {
        double result;
        if (side == Hand.kLeft) {
            result = laccumulator;
        } else {
            result = raccumulator;
        }
        return result;
    }

    private void resetAccumulator() {
        laccumulator = 0;
        raccumulator = 0;
    }

    public void accumulate() {
        double dt;
        long currentTime = System.currentTimeMillis();
        if (previousTime == 0) {
            dt = 0;
        } else {
            dt = (currentTime - previousTime) / 1000.0;
        }
        previousTime = currentTime;

        laccumulator += dt * rotationRate(Hand.kLeft) * wheelRadius * gearRatio;
        raccumulator += dt * rotationRate(Hand.kRight) * wheelRadius * gearRatio;
        SmartDashboard.putNumber("Accumulator", laccumulator);
        SmartDashboard.putNumber("Rotation Rate", rotationRate(Hand.kLeft));
    }

    public void switchColor() {
        if(color == .85){
            color = .61;
        }
        else{
            color = .85;
        }
        ledDriver.set(color);
        System.out.println("Setting Color!");
        SmartDashboard.putNumber("blinkin Color", color);
    }

    public abstract void setWheelSpeed(double leftSpeed, double rightSpeed);

    // radians / second
    protected abstract double rotationRate(Hand hand);
}