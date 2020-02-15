package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class nullSystem {
    public static class nullDriveTrain extends DriveTrain{
        public nullDriveTrain(colorSensingWheelBot robot){
            super(robot);
        }
        public void operate(){

        }

        public void setWheelSpeed(double l, double r){

        }

        public double rotationRate(Hand h){
            return 0;
        }
    }

    public static class nullBallSystem extends ballSystem {
        public nullBallSystem(colorSensingWheelBot robot){
            super(robot);
        }
        public void operate(){

        }
    }

    public static class nullVisionSystem extends visionSystem {
        public nullVisionSystem(colorSensingWheelBot robot){
            super(robot);
        }
        public void operate(){
            
        }
    }

    public static class nullLifterSystem extends lifterSystem {
        public nullLifterSystem (colorSensingWheelBot robot){
            super(robot);
        }
        public void operate(){
            
        }
    }

    public static class nullSpinnerSystem extends spinnerSystem {
        public nullSpinnerSystem(colorSensingWheelBot robot){
            super(robot);
        }
        public void operate(){
            
        }
    }

}

    
