package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class wheelBotDriveTrain extends DriveTrain {

    // These are the two motors characteristic of a treadbot
    private TalonSRX l1Talon;
    private TalonSRX r1Talon;
    private TalonSRX l2Talon;
    private TalonSRX r2Talon;
    

    public wheelBotDriveTrain(colorSensingWheelBot theRobot) {
        super(theRobot);
        l1Talon = new TalonSRX(1);
        l2Talon = new TalonSRX(2);

        r1Talon = new TalonSRX(3);
        r2Talon = new TalonSRX(4);

        l1Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        ErrorCode encoderCode1 = l1Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        r1Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        ErrorCode encoderCode2 = r1Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        SmartDashboard.putString("Quad1Error", encoderCode1.toString());
        SmartDashboard.putString("Quad2Error", encoderCode2.toString());
        

    }

    @Override
    public void setWheelSpeed(double leftSpeed, double rightSpeed) {
        l1Talon.set(ControlMode.PercentOutput, leftSpeed);
        l2Talon.set(ControlMode.PercentOutput, leftSpeed);
        r1Talon.set(ControlMode.PercentOutput, rightSpeed);
        r2Talon.set(ControlMode.PercentOutput, rightSpeed);
    }

    // radians / sec
    @Override
    protected double rotationRate(Hand side) {

        double unitsPers;
        if (side == Hand.kLeft) {
            unitsPers = l2Talon.getSelectedSensorVelocity(0) * 10;
        } else {
            unitsPers = r1Talon.getSelectedSensorVelocity(0) * 10;
        }

        // SmartDashboard.putNumber("RotationRate", unitsPers/unitsPerRadian);

        return unitsPers / unitsPerRadian;
    }

}