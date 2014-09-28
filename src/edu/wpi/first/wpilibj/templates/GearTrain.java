/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Joystick;

/**
 * Will be used for drive and climbing
 * 
 * @author Michael C., Sarang M.
 */
public class GearTrain extends RobotDrive {
    //Joystick leftStick, rightStick;
    private final double MAX_DRIVE_VALUE = 1; // desired Max Value of speed/joystick while driving
    private final double MAX_CLIMB_VALUE = 0.4; // desired Max Value of speed/joystick while climbing
    private final int INVERSION = -1;
    
    double TIME_INTERVAL = 0.1;
    double velocity=0;
    double VELOCITY_TO_JAG_CONSTANT;
    double jagIncrement;
    
    double path[];
    double timeInterval[];
    
    double filterLSpeed;
    double filterRSpeed;
    
    double driveAlpha;
    
    Timer clock;
    
    /**
     * Constructor, utilizes RobotDrive
     */
    public GearTrain (){
        super(HWR.LEFT_MOTOR, HWR.RIGHT_MOTOR);
        clock = new Timer();
        //leftStick = passL;
        //rightStick = passR;
    }
    
    /**
     * Sets the speed of both sides using input from Driver Station, used for driving
     */
    public void driveMode(){ 
        driveAlpha = DriverStation.getDouble("DriveAlpha");
        //double lSpeed = leftStick.getY();
        //double rSpeed = rightStick.getY();
        double lSpeed = DriverStation.leftStick.getRawAxis(DriverStation.kYAxis);
        double rSpeed = DriverStation.rightStick.getRawAxis(DriverStation.kYAxis);
                
        lSpeed = quadScale(lSpeed)*INVERSION;
        rSpeed = quadScale(rSpeed)*INVERSION;
        
        filterLSpeed = (1-driveAlpha) * lSpeed + driveAlpha*filterLSpeed;
        filterRSpeed = (1-driveAlpha) * rSpeed + driveAlpha*filterRSpeed;
        
        SmartDashboard.putNumber("lSpeed", filterLSpeed);
        SmartDashboard.putNumber("rSpeed", filterRSpeed);
        
        setLeftRightMotorOutputs(filterLSpeed, filterRSpeed);
    }
    
    /**
     * Sets the speed of both sides using input from Driver Station, used for climbing
     */
    public void climbMode(){
        //double lSpeed = leftStick.getY();
        //double rSpeed = rightStick.getY();
        //double lSpeed = DriverStation.leftStick.getRawAxis(DriverStation.kYAxis);
        //double rSpeed = DriverStation.rightStick.getRawAxis(DriverStation.kYAxis);
        
        //lSpeed = fractionScale(lSpeed);
        //rSpeed = fractionScale(rSpeed);
        double speed = DriverStation.rightStick.getRawAxis(DriverStation.kYAxis);
        //speed = fractionScale(speed);
        
        //FOR TESTING: COMMENT OUT ONE LINE, THEN THE OTHER TO MAKE SURE MOTORS ARE GOING IN SAME DIRECTION
        setMotorSpeed(-speed,-speed);
        //setMotorSpeed(speed, 0.0);
        //setLeftRightMotorOutputs(speed, speed);
    }
    /**
     * Sets the speed of both sides
     * @param lSpeed speed of the left side
     * @param rSpeed speed of the right side
     */
    public void setMotorSpeed(double lSpeed, double rSpeed){
        setLeftRightMotorOutputs(lSpeed, rSpeed);
    }
    
    /**
     * Sets the speed of both sides using input from Driver Station, not scaled 
     * 
     */
    public void setMotorSpeed(){
        double lSpeed = DriverStation.leftStick.getRawAxis(DriverStation.kYAxis);
        double rSpeed = DriverStation.rightStick.getRawAxis(DriverStation.kYAxis);
        setLeftRightMotorOutputs(lSpeed, rSpeed);
    }
    
    /**
     * Based off of 2012 quadScale method, scales joystick input to be easier to use.
     * @param toScale the value to be scaled
     * @return the scaled value
     */
    private double quadScale(double toScale){
        toScale = toScale*MAX_DRIVE_VALUE;
        
        if (toScale < 0) {
            toScale *= -toScale;
        } else {
            toScale *= toScale;
        }
        return toScale;
    }
    
    /**
     * Scales joystick input to half original input. To be used for climbing.
     * @param toScale value to be scaled
     * @return scaled value
     */
    private double fractionScale(double toScale){
        toScale = toScale*MAX_CLIMB_VALUE;
        return toScale;
    }
    
    /**
     * assumes that you go 1/4 of the distance while accelerating
     * half while at a constant speed
     * then 1/4 while deccelerate
     * @param distance
     * @param time 
     */
    public void driveDistance (double distance, double time) {
        //
        double accelTime = time/3;
        double constantTime = time/3;
        double deccelTime = time/3;
        
        double accelDistance = distance/4;
        double constantDistance = distance/2;
        double deccelDistance = distance/4;
        
        double accelVelocity = 2*accelDistance/accelTime;
        double constantVelocity = constantDistance/constantTime;
        double deccelVelocity = -2*deccelDistance/deccelTime;
        
        double accel = accelVelocity/accelTime;
        double constant = 0;
        double deccel = deccelVelocity/deccelTime;
        
        path[0] = accel;
        path[1] = constant;
        path[2] = deccel;
        
        timeInterval[0] = accelTime;
        timeInterval[1] = constantTime;
        timeInterval[2]  = deccelTime;   
    }
    public void setAccel(double accel , double deltaT) {
        clock.start();
        while (clock.get() <= deltaT) {
        velocity = velocity+(accel/(1/TIME_INTERVAL));
        jagIncrement = velocity*VELOCITY_TO_JAG_CONSTANT; // needs to be calculated
        setMotorSpeed(jagIncrement,jagIncrement);
        Timer.delay(TIME_INTERVAL);
        }
        clock.stop();
       
    }
    /**
     * Should use path and timeInterval arrays calculated by driveDistance
     * @param path
     * @param time 
     */
    public void motionProfiling(double path[], double time[]) {
        clock.reset();
        setAccel(path[0], time[0]); // 1, 1
        clock.reset();
        setAccel(path[1], time[1]); // 0,1
        clock.reset();
        setAccel(path[2], time[2]);//  -1, 1
    }
}
