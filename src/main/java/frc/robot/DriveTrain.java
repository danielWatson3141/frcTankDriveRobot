package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DriveTrain extends Subsystem {

    double laccumulator = 0;
    double raccumulator = 0;
    long previousTime = 0; // ms since Jan 1 1970

    private Spark ledDriver;


    static final double gearRatio = 1 / 6.0; // unitless
    static final double wheelRadius = .15; // meters

    static final double unitsPerRotation = 360;
    static final double unitsPerRadian = unitsPerRotation / (2 * Math.PI);

    static final Double STEERING_STRENGTH = .5;

    public DriveTrain() {

    }

    public DriveTrain(colorSensingWheelBot theRobot) {
        super(theRobot);
        ledDriver = new Spark(5);
    }

    public void operate() {
        double leftSpeed;
        double rightSpeed;

        double xAxis = controller.getRawAxis(1) * STEERING_STRENGTH; // dampen the x axis to improve handling
        double yAxis = controller.getRawAxis(0);

        leftSpeed = (xAxis + yAxis) * .5;
        rightSpeed = (-xAxis + yAxis) * .5;

        setWheelSpeed(leftSpeed, rightSpeed);
        setColor();
        accumulate();
    }

    // theta: degrees
    // turn at half speed
    public void turn(double theta) {
        try {
            resetAccumulator();
            double leftSpeed, rightSpeed;
            distanceTravelled(Hand.kLeft);

            leftSpeed = rightSpeed = Math.signum(theta) * STEERING_STRENGTH;
            setWheelSpeed(leftSpeed, rightSpeed);

            // .5 is because we have 2 "wheels" turning opposite directions
            double distance = .5 * theta * wheelRadius * Math.PI / 180;

            while (distanceTravelled(Hand.kLeft) < distance) {
                accumulate();
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
            leftSpeed = .5 * Math.signum(distance);
            rightSpeed = -.5 * Math.signum(distance);
            setWheelSpeed(leftSpeed, rightSpeed);

            while (distanceTravelled(Hand.kLeft) < distance) {
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

    public void setColor() {
        int count = 0;
        if (controller.getStartButtonPressed()) {
            count++;
            if (count == 0)
                ledDriver.set(0.87);
            else if (count == 1)
                ledDriver.set(0.61);
            else if (count > 1)
                count = 0;

        }
    }

    public abstract void setWheelSpeed(double leftSpeed, double rightSpeed);

    // radians / second
    protected abstract double rotationRate(Hand hand);
}