package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class treadBotDriveTrain extends DriveTrain {

    // These are the two motors characteristic of a treadbot
    private TalonSRX l1Talon;
    private TalonSRX r1Talon;
    
    public treadBotDriveTrain(colorSensingWheelBot theRobot){
        super(theRobot);
        l1Talon = new TalonSRX(1);
        r1Talon = new TalonSRX(2);
        

        l1Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        r1Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    }

    @Override
    public void setWheelSpeed(double leftSpeed, double rightSpeed) {
        l1Talon.set(ControlMode.PercentOutput, leftSpeed);
        r1Talon.set(ControlMode.PercentOutput, rightSpeed);
        
    }

    //radians / sec
    @Override
    protected double rotationRate(Hand side){

        double unitsPers;
        if( side == Hand.kLeft){
            unitsPers =  l1Talon.getSelectedSensorVelocity(0)*10;
        }else{
            unitsPers =  r1Talon.getSelectedSensorVelocity(0)*10;
        }

        SmartDashboard.putNumber("RotationRate", unitsPers/unitsPerRadian);

        return unitsPers / unitsPerRadian;
    }
}