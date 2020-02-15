package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class nullSystem {
    public static class nullDriveTrain extends DriveTrain{
        
        public void operate(){

        }

        public void setWheelSpeed(double l, double r){

        }

        public double rotationRate(Hand h){
            return 0;
        }
    }

    public static class nullBallSystem extends ballSystem {
        public void operate(){

        }
    }

    public static class nullVisionSystem extends visionSystem {
        public void operate(){
            
        }
    }

    public static class nullLifterSystem extends lifterSystem {
        public void operate(){
            
        }
    }

    public static class nullSpinnerSystem extends spinnerSystem {
        public void operate(){
            
        }
    }

}

    
