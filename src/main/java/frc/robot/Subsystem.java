package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public abstract class Subsystem{

    public boolean active = false;
    public colorSensingWheelBot robot;
    public XboxController controller;

    public Subsystem(){
        boolean active = false;
    }

    public Subsystem(colorSensingWheelBot theRobot){
        robot = theRobot;
        controller = robot.myController;
        active = false;
    }

    public void activate(){
        active = true;
    }
    public void deactivate(){
        active = false;
    }

    public void act(){
        if(active){
            try{
                operate();
            } catch (Exception e){
                e.printStackTrace();
                System.out.println("Problem occured in "+this.getClass()+": \n"+"System shut down!");
                active = false;
            }
        }
    }

    public abstract void operate();
}