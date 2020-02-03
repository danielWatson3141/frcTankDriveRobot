package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;

public class ballSystem extends Subsystem {

    // This is the motor for the belt and the servo motor for the ball mechanism
    private TalonSRX chain;
    private Servo ballServo;

    public ballSystem(colorSensingWheelBot theRobot) {
        super(theRobot);

        // motor for ball grabber
        chain = new TalonSRX(5);

        // servo for the ball dumper
        ballServo = new Servo(2);
    }

    @Override
    public void operate() {
        if (controller.getAButton()) {
            chain.set(ControlMode.PercentOutput, 1);
        }
    }

    public void openGate() {
        ballServo.set(90);
        chain.set(ControlMode.PercentOutput, 1);
    }

    public void closeGate() {
        ballServo.set(0);
        chain.set(ControlMode.PercentOutput, 1);
    }
}