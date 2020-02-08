package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;

public class visionSystem extends Subsystem {

    public Ultrasonic front;
    public Ultrasonic rear;

    // This is network table data for the limelight
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ts;
    private double xPos;
    private double yPos;
    private double area;
    private boolean lightState;
    private double distanceShort;
    private double distanceLong;
    private double xOffset;
    private double yOffset;
    private NetworkTableEntry tshort;
    private NetworkTableEntry tlong;
    private double bigC;
    private static final double WIDTH_OF_TARGET = .86;
    private static final double HEIGHT_OF_TARGET = .254;
    private static final double RATIO_OF_DEGREES = 20;

    public visionSystem(colorSensingWheelBot theRobot) {
        super(theRobot);
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        ts = table.getEntry("ts");
        tshort = table.getEntry("tshort");
        tlong = table.getEntry("tlong");
        front = new Ultrasonic(0,0);
        rear = new Ultrasonic(1,1);
    }

    @Override
    public void operate() {

        // Provides smartboard data for the limelight
        xPos = tx.getDouble(0.0);
        yPos = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        distanceShort = HEIGHT_OF_TARGET * Math.tan(tshort.getDouble(0.0) * RATIO_OF_DEGREES);
        distanceLong = HEIGHT_OF_TARGET * Math.tan(tlong.getDouble(0.0) * RATIO_OF_DEGREES);
        bigC = lawOfCosines(distanceShort, WIDTH_OF_TARGET, distanceLong);
        lightState = tv.getBoolean(false);
        xOffset = distanceShort * Math.cos(bigC);
        yOffset = distanceShort * Math.sin(bigC);
        SmartDashboard.putNumber("LimelightX", xPos);
        SmartDashboard.putNumber("LimelightY", yPos);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("Target Acquired?", lightState);
        SmartDashboard.putNumber("Distance Short", distanceShort);
        SmartDashboard.putNumber("Distance Long", distanceLong);
        SmartDashboard.putNumber("xOffset", xOffset);
        SmartDashboard.putNumber("yOffset", yOffset);
    }

    //whether the vision system has a lock on the target
    public boolean targetAcquired() {
        return lightState;
    }

    //horizontal offset from target.
    //defined as distance from the center axis of the target
    //0 implies that the robot is lined up with center axis of the target
    //in meters
    public double targetXOffset() {
        return xOffset;
    }

    //"vertical" offset from the target.
    //distance from the plane of the target (the wall)
    //0 implies that the robot is against the wall
    //in meters
    public double targetYOffset() {
        return yOffset;
    }

    //the horizontal component of the robot's offset angle with respect to the target
    //this is the angle the robot would need to turn to be faced directly at the target
    //in degrees
    public double targetAngleOffset(){
        return tx.getDouble(0);
    }

    public double lawOfCosines(double a, double b, double c) {
        return Math.acos((Math.pow(c, 2) - Math.pow(b, 2) - Math.pow(a, 2)) / (-2 * a * b));
    }

    public boolean obstacleDetectedFront(){
        //front.getRangeMM();
        if (front.getRangeMM() <= 20)
            return true;
        return false;
    }

    public boolean obstacleDetectedRear(){
        if (rear.getRangeMM() <= 20)
            return true;
        return false;
    }

}