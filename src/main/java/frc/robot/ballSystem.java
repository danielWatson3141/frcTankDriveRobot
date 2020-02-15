package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;

public class ballSystem extends Subsystem {

    // This is the motor for the belt and the servo motor for the ball mechanism
    public TalonSRX belt;
    public Servo ballServo;
    public static final double servoOffsetAngle = 50;

    public ballSystem(){}

    public ballSystem(colorSensingWheelBot theRobot) {
        super(theRobot);

        // motor for ball grabber
        belt = new TalonSRX(6);

        // servo for the ball dumper
        ballServo = new Servo(2);
    }

    @Override
    public void operate() {
        // makes the ball collect motor run
        if (controller.getAButton()) {
            belt.set(ControlMode.PercentOutput, .6);
        } else {
            belt.set(ControlMode.PercentOutput, 0);
        }
    }

    public void openGate() {
        // opens the gate to release the balls
        ballServo.set(.55);
        belt.set(ControlMode.PercentOutput, .6);
    }

    public void closeGate() {
        // closes the gate
        ballServo.set(.94);
        belt.set(ControlMode.PercentOutput, 0);
    }
}