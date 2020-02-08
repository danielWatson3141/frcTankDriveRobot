package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class visionSystem extends Subsystem {

    // This is network table data for the limelight
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    public visionSystem(colorSensingWheelBot theRobot) {
        super(theRobot);
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    @Override
    public void operate() {

        // Provides smartboard data for the limelight
        double xPos = tx.getDouble(0.0);
        double yPos = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", xPos);
        SmartDashboard.putNumber("LimelightY", yPos);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    //whether the vision system has a lock on the target
    public boolean targetAcquired(){
        return false;
    }

    //horizontal offset from target.
    //defined as distance from the center axis of the target
    //0 implies that the robot is lined up with center axis of the target
    //in meters
    public double targetXOffset(){
        return 0;
    }

    //"vertical" offset from the target.
    //distance from the plane of the target (the wall)
    //0 implies that the robot is against the wall
    //in meters
    public double targetYOffset(){
        return 0;
    }

    //the horizontal component of the robot's offset angle with respect to the target
    //this is the angle the robot would need to turn to be faced directly at the target
    //in degrees
    public double targetAngleOffset(){
        return 0;
    }



}