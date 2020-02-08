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

        setWheelSpeed(leftSpeed, rightSpeed);
    }

    // theta: degrees
    // turn at half speed
    public void turn(double theta) {

        double totalDistance = 0;
        double leftSpeed, rightSpeed;
        distanceTravelled(Hand.kLeft);

        leftSpeed = rightSpeed = Math.signum(theta) * .5;
        setWheelSpeed(leftSpeed, rightSpeed);

        double distance = theta * wheelRadius * Math.PI / 180;

        while (totalDistance < distance) {
            accumulate();
            totalDistance += distanceTravelled(Hand.kLeft);
        }

        leftSpeed = rightSpeed = .0;
        setWheelSpeed(leftSpeed, rightSpeed);
    }

    // distance: meters
    // move at half speed
    public void move(double distance) {
        double totalDistance = 0;
        double leftSpeed, rightSpeed;
        distanceTravelled(Hand.kLeft);
        leftSpeed = .5 * Math.signum(distance);
        rightSpeed = -.5 * Math.signum(distance);
        setWheelSpeed(leftSpeed, rightSpeed);

        while (totalDistance < distance) {
            accumulate();
            totalDistance += distanceTravelled(Hand.kLeft);
            SmartDashboard.putNumber("distance travelled", totalDistance);

        }

        leftSpeed = rightSpeed = .0;
        setWheelSpeed(leftSpeed, rightSpeed);
    }

    private double distanceTravelled(Hand side) {
        double result;
        if (side == Hand.kLeft) {
            result = laccumulator;
            laccumulator = 0;
        } else {
            result = raccumulator;
            raccumulator = 0;
        }
        return result;
    }

    private void accumulate() {
        double dt;
        long currentTime;
        if (previousTime == 0) {
            dt = 0;
        } else {
            currentTime = System.currentTimeMillis();
            dt = (currentTime - previousTime) / 1000.0;
            previousTime = currentTime;
        }

        laccumulator += dt * rotationRate(Hand.kLeft) * wheelRadius * gearRatio;
        raccumulator += dt * rotationRate(Hand.kRight) * wheelRadius * gearRatio;
        SmartDashboard.putNumber("Left Accumulator", laccumulator);

    }

    public abstract void setWheelSpeed(double leftSpeed, double rightSpeed);

    // radians / second
    protected abstract double rotationRate(Hand hand);
}