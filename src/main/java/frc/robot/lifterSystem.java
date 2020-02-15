package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class lifterSystem extends Subsystem {

    // This is the motors and hall effect sensors for the lifter mechanisms
    public TalonSRX extTalon;
    public TalonSRX ropeTalon;
    public DigitalInput sensorTop;
    public DigitalInput sensorBottom;

    public lifterSystem(){

    }

    public lifterSystem(colorSensingWheelBot theRobot) {
        super(theRobot);

        // motor that extends arm
        extTalon = new TalonSRX(7);

        // motor that retracts rope
        ropeTalon = new TalonSRX(8);

        // sensors that tell us if we're at the top or bottom of the shaft
        sensorTop = new DigitalInput(0);
        sensorBottom = new DigitalInput(1);

    }

    @Override
    public void operate() {
        double leftTrigger = controller.getTriggerAxis(Hand.kLeft);
        double rightTrigger = controller.getTriggerAxis(Hand.kRight);
        double speedRequest = (rightTrigger-leftTrigger)*.5;

        if(sensorTop.get()){
            speedRequest = Math.min(speedRequest, 0);
        }

        if(sensorBottom.get()){
            speedRequest = Math.max(speedRequest, 0);
        }

        extTalon.set(ControlMode.PercentOutput, speedRequest);
    }

    public void extend() {
        if (sensorTop.get()) {
            extTalon.set(ControlMode.PercentOutput, 0);
        } else {
            extTalon.set(ControlMode.PercentOutput, .5);
        }
    }

    public void retract() {
        ropeTalon.set(ControlMode.PercentOutput,controller.getRawAxis(5));
        if (!sensorBottom.get()) {
            extTalon.set(ControlMode.PercentOutput, 0);
        } else {
            extTalon.set(ControlMode.PercentOutput,-.5);
        }

    }
}