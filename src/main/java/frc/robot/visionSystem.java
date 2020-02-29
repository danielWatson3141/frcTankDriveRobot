package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

import java.awt.geom.Point2D;

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
    private double lightState;
    private double distanceShort;
    private double distanceLong;
    private double xOffset;
    private double yOffset;
    private NetworkTableEntry tshort;
    private NetworkTableEntry tlong;
    private NetworkTableEntry tcornx;
    private NetworkTableEntry tcorny;
    
    private double bigC;
    private static final double WIDTH_OF_TARGET = .86;
    private static final double HEIGHT_OF_TARGET = .254;
    private static final double RATIO_OF_HDEGREES = 5.369;
    private static final double RATIO_OF_LDEGREES = 4.828;

    public visionSystem(){}

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
        tcornx = table.getEntry("tcornx");
        tcorny = table.getEntry("tcorny");
        CameraServer.getInstance().startAutomaticCapture();
        //front = new Ultrasonic(0,0);
        //rear = new Ultrasonic(1,1);
    }

    private static final double ELEV = .05;
    @Override
    public void operate() {
        // Provides smartboard data for the limelight
        xPos = tx.getDouble(0.0);
        yPos = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        double tShortAngle = tshort.getDouble(0)/RATIO_OF_LDEGREES;
        double tLongAngle = tlong.getDouble(0)/RATIO_OF_HDEGREES;
        double h = .27;

        distanceShort = h/Math.tan(degToRad(tShortAngle));
        distanceLong = 0;

        double observedLongOverShort = tLongAngle / tShortAngle;
        double trueLongOverShort = 20.5 / 10 * .6;

        SmartDashboard.putNumber("observed ", observedLongOverShort);
        double ratioObservedReal = observedLongOverShort / trueLongOverShort;
        SmartDashboard.putNumber("ratioObservedReal", ratioObservedReal);
        
        if(ratioObservedReal>=1)
            bigC = 0;
        else
            bigC = 90-Math.toDegrees(Math.asin(ratioObservedReal));
        
        if(bigC < 30)
            bigC = 0;

        
        lightState = tv.getDouble(0);
        xOffset = distanceShort * Math.cos(bigC);
        yOffset = distanceShort * Math.sin(bigC);

        int direction = findSide(tcornx.getDoubleArray(new Double[]{0.0}), tcorny.getDoubleArray(new Double[]{0.0}));

        bigC*=direction;

        SmartDashboard.putNumber("side", direction);
        SmartDashboard.putNumber("angleOffset", bigC);
        SmartDashboard.putNumber("LimelightX", xPos);
        SmartDashboard.putNumber("LimelightY", yPos);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Target Acquired", lightState);
        SmartDashboard.putNumber("tshort - angle", tShortAngle);
        SmartDashboard.putNumber("tlong - angle", tLongAngle);
        SmartDashboard.putNumber("Distance Short", distanceShort);
        SmartDashboard.putNumber("Distance Long", distanceLong);
        SmartDashboard.putNumber("xOffset", xOffset);
        SmartDashboard.putNumber("yOffset", yOffset);
        SmartDashboard.putNumber("skew" , ts.getDouble(0));
    }

    private int findSide(Double[] cornerXs, Double[] cornerYs) {

        System.out.println("finding side");

        SmartDashboard.putNumberArray("cornerXs", cornerXs);
        SmartDashboard.putNumberArray("cornerYs", cornerYs);
        SmartDashboard.putNumber("Xs size",cornerXs.length);
        if(cornerXs.length == 1){
            return 0;
        }

        Point2D.Double[] points = new Point2D.Double[4];
        for(int i=0; i<4; i++){
            points[i] = new Point2D.Double(cornerXs[i], cornerYs[i]);
        }

        for(int i=0; i<4; i++){
            for(int j =0; j<3; j++){
                if(points[j].getX() > points[j+1].getX()){
                    Point2D.Double temp = points[j];
                    points[j] = points[j+1];
                    points[j+1] = temp;
                }
            }
        }

        if(points[0].getY()<points[1].getY()){
            Point2D.Double temp = points[0];
                    points[0] = points[1];
                    points[1] = temp;
        }

        
        if(points[2].getY()<points[3].getY()){
            Point2D.Double temp = points[2];
                    points[2] = points[3];
                    points[3] = temp;
        }
        double distLeft = points[0].distance(points[1]);
        double distRight = points[2].distance(points[3]);

        SmartDashboard.putNumber("distLeft", distLeft);
        SmartDashboard.putNumber("distRight", distRight);

        return distLeft < distRight ? 1 : -1;
    }

    private double degToRad(double deg) {
        return Math.PI*deg/180;
    }

    //whether the vision system has a lock on the target
    public boolean targetAcquired() {
        return lightState == 1;
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