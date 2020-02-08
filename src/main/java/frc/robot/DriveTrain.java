package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DriveTrain extends Subsystem {

    double laccumulator = 0;
    double raccumulator = 0;
    long previousTime = 0; // ms since Jan 1 1970
    static final double gearRatio = 1 / 6.0; // unitless
    static final double wheelRadius = .15; // meters

    static final double unitsPerRotation = 360;
    static final double unitsPerRadian = unitsPerRotation / (2 * Math.PI);

    public DriveTrain(colorSensingWheelBot theRobot) {
        super(theRobot);
    }

    public void operate() {
        double leftSpeed;
        double rightSpeed;

        leftSpeed = (controller.getRawAxis(1) + controller.getRawAxis(0)) * .5;
        rightSpeed = (-controller.getRawAxis(1) + controller.getRawAxis(0)) * .5;

        accumulate();

        setWheelSpeed(leftSpeed, rightSpeed);
    }

    // theta: degrees
    // turn at half speed
    public void turn(double theta) {
        resetAccumulator();
        double leftSpeed, rightSpeed;
        distanceTravelled(Hand.kLeft);

        leftSpeed = rightSpeed = Math.signum(theta) * .5;
        setWheelSpeed(leftSpeed, rightSpeed);

        double distance = theta * wheelRadius * Math.PI / 180;

        while (distanceTravelled(Hand.kLeft) < distance) {
            accumulate();
        }

        leftSpeed = rightSpeed = .0;
        setWheelSpeed(leftSpeed, rightSpeed);
    }

    // distance: meters
    // move at half speed
    public void move(double distance) {
        resetAccumulator();
        double leftSpeed, rightSpeed;
        distanceTravelled(Hand.kLeft);
        leftSpeed = .5 * Math.signum(distance);
        rightSpeed = -.5 * Math.signum(distance);
        setWheelSpeed(leftSpeed, rightSpeed);

        while (distanceTravelled(Hand.kLeft) < distance) {
            accumulate();
        }

        leftSpeed = rightSpeed = .0;
        setWheelSpeed(leftSpeed, rightSpeed);
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

    public abstract void setWheelSpeed(double leftSpeed, double rightSpeed);

    // radians / second
    protected abstract double rotationRate(Hand hand);
}