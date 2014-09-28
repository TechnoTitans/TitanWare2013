/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Double;
/**
 *  Used for Air Control
 * @author Rohan Doshi, Priunsh Nagru 2013
 */
public class Pneumatics {
    // DEFINING THE COMPRESSOR
    private volatile Compressor comp;
    
   
    //private volatile Solenoid gearShift;
    private volatile Solenoid winch;
    //private Solenoid test;
    private Servo leftGearShift;
    private Servo rightGearShift;
   
    
    private final boolean RETRACT = false;
    private final boolean EXTEND = true;
    boolean lastRun = false;
    boolean mode = RETRACT;
    
    //private volatile Solenoid leftPiston;
    //private volatile Solenoid rightPiston;
    
    private Solenoid climbPiston;
    private volatile Solenoid shooterAngle;
    
    private  double SERVO_MAX = 1.0;
    public  double HIGH_GEAR_SERVO_VALUE = 0.8;
    public  double LEFT_LOW_GEAR_SERVO_VALUE = 0.2;
    public  double RIGHT_LOW_GEAR_SERVO_VALUE = 0.0;
    public  double LEFT_NEUTRAL_GEAR_SERVO_VALUE = .7;
    public  double RIGHT_NEUTRAL_GEAR_SERVO_VALUE = 0.5;
    public double lastServoValue = 0.0;
    private final boolean mirror = false;
    private boolean shotHigh = true;
    public boolean winchIn = false;
    public String gearState;
    

    
   
    public Pneumatics(){
        // SOLENOID DECLARATIONS
        //gearShift = new Solenoid(HWR.SOLENOID_SLOT, HWR.GEAR_SHIFT_SOLENOID);
        climbPiston = new Solenoid(HWR.SOLENOID_SLOT, HWR.CLIMB_PISTON);
        comp = new Compressor(HWR.COMPRESSOR_PRESSURE_SWITCH, HWR.COMPRESSOR_RELAY);
        leftGearShift = new Servo(HWR.SERVO_1_SLOT);
        rightGearShift = new Servo(HWR.SERVO_2_SLOT);
        winch = new Solenoid(HWR.SOLENOID_SLOT, HWR.WINCH_SOLENOID);
        shooterAngle = new Solenoid(HWR.SOLENOID_SLOT, HWR.SHOOTER_ANGLE_SOLENOID);
        //comp.start();
        //test = new Solenoid(8);
        //System.out.println("Pnumatics called");
    }    
          
    public boolean getPressureSwitch() {
        return comp.getPressureSwitchValue();
    }
           
    /**
     * Start the Compressor
     */
    public void startCompressor() {
        comp.start();
        System.out.println("Compressor Started");
    }
    
    /**
     * Stop the Compressor
     */
    public void stopCompressor() {
        comp.stop();
        //System.out.println("Compressor Stopped");
    }
    
    /**
     * Shift gears up or down
     * 
     * @param shiftUp true or false
     */
//    public void shiftGears(boolean shiftUp){
//        gearShift.set(shiftUp);
//        //System.out.println("Gear Shifted");
//    }
    
    /**
     * Turns winch on or off
     * @param winchOn true = climbing mode
     */
    public void shiftMode(boolean winchOn){
        winch.set(winchOn);
        System.out.println("Winch" + winchOn);
    }
    
    /**
     * Shifts gear by setting servo value to parameter
     * @param shiftUp should be .2, .8, or .5
     */
    public void shiftGears(double left, double right){
        leftGearShift.set(left);
        if(mirror){
            rightGearShift.set(HIGH_GEAR_SERVO_VALUE-right);
        } else if(!mirror) {
            rightGearShift.set(right);
        }
        
        lastServoValue = left;
        //System.out.println("Gear Shifted");
    }
    
    
    /**
     * Checks if gearShift solenoid is set to true.
     * 
     * Wording of function name may be changed based 
     * upon how the system is built.
     * @return
     */
//    public boolean isGearHigh(){
//        return gearShift.get();
//    }
    
    /**
     * Checks if winch Solenoid is set to true
     * @return if winch solenoid is set to true or not
     */
    public boolean isWinchOn(){
        return winch.get();
    }
    
    /**
     * Checks angle of servo to find what gear it is in
     * @return String describing what gear we are in
     */
    public String checkGear(){

        
        if(leftGearShift.get()==HIGH_GEAR_SERVO_VALUE||rightGearShift.get()==HIGH_GEAR_SERVO_VALUE){
            System.out.println("Left or right motor is in high gear");
            gearState = "High Gear";

        }
         
//        if(leftGearShift.get()>LEFT_NEUTRAL_GEAR_SERVO_VALUE||rightGearShift.get()>RIGHT_NEUTRAL_GEAR_SERVO_VALUE){
//            System.out.println("Left or right motor is in high gear");
//            gearState = "High Gear";
//
//        }
        
        else if(leftGearShift.get()==LEFT_LOW_GEAR_SERVO_VALUE||rightGearShift.get()==RIGHT_LOW_GEAR_SERVO_VALUE){
            System.out.println("Left or right motor is in low gear");
            gearState = "Low Gear";
        }
        
//        else if(leftGearShift.get()<LEFT_NEUTRAL_GEAR_SERVO_VALUE||rightGearShift.get()<RIGHT_NEUTRAL_GEAR_SERVO_VALUE){
//            System.out.println("Left or right motor is in low gear");
//            gearState = "Low Gear";
//        } 
        
        else if(leftGearShift.get()==LEFT_NEUTRAL_GEAR_SERVO_VALUE||rightGearShift.get()==RIGHT_NEUTRAL_GEAR_SERVO_VALUE){
            System.out.println("Left or right motor is in neutral gear");
            gearState = "Neutral Gear";
        }
        else{
            gearState =  "No Gear";
        }
        return gearState;
    }
   
    /**
     * Extends the piston if the input is true 
     * @param isRetracted if true the piston is extended, if false the 
     * piston is retracted
     */
   public void extendPiston(boolean isRetracted){
       //leftPiston.set(isRetracted);
       //rightPiston.set(isRetracted);
       climbPiston.set(isRetracted);
   }
   
   /**
    * Changes the angle between two set positions, using Pneumatics.
    * @param isHigh whether or not the shooter is supposed to be aiming high or not.
    */
   public void shotAngle(boolean isHigh){
       shooterAngle.set(isHigh);
   }
            
   public String getGearState () {
       return gearState;
   }

   public void calibrateServos () {
       double rightPosition = DriverStation.rightStick.getAxis(Joystick.AxisType.kZ);
       double leftPosition = DriverStation.leftStick.getAxis(Joystick.AxisType.kZ);
       rightPosition = (rightPosition-1)/-2;
       leftPosition = (leftPosition-1)/-2;
       shiftGears(leftPosition,rightPosition);
       SmartDashboard.putNumber("rPosition", rightPosition);
       SmartDashboard.putNumber("lPosition", leftPosition);
   }
    /**
     * Monitor joystick inputs for various pneumatic stuff.
     * 
     * It can be run normally, or as a thread if so desired.
     * 
     * The pneumatics works when the right trigger is held down only and
     * retracts when the trigger is let go. 
     */
    public void run(){  
        
//        SmartDashboard.putBoolean("Pressure Switch", getPressureSwitch());
//        SmartDashboard.putBoolean("Compressor Enabled", comp.enabled());
//        
//        HIGH_GEAR_SERVO_VALUE = (DriverStation.getDouble("High"));
//        LEFT_LOW_GEAR_SERVO_VALUE = DriverStation.getDouble("LeftLow");
//        RIGHT_LOW_GEAR_SERVO_VALUE = DriverStation.getDouble("RightLow");
//        LEFT_NEUTRAL_GEAR_SERVO_VALUE = DriverStation.getDouble("LeftN");
//        RIGHT_NEUTRAL_GEAR_SERVO_VALUE = DriverStation.getDouble("RightN");
//        
//        SmartDashboard.putNumber("High",HIGH_GEAR_SERVO_VALUE );
//        SmartDashboard.putNumber("LeftLow", LEFT_LOW_GEAR_SERVO_VALUE);
//        SmartDashboard.putNumber("RighLow", RIGHT_LOW_GEAR_SERVO_VALUE);
//        SmartDashboard.putNumber("LeftN", LEFT_NEUTRAL_GEAR_SERVO_VALUE);  
//        SmartDashboard.putNumber("RightN", RIGHT_NEUTRAL_GEAR_SERVO_VALUE);
//         
//        SmartDashboard.putNumber("RServo Position", rightGearShift.get());
//        SmartDashboard.putNumber("LServo Position", leftGearShift.get());
        if(DriverStation.rightStick.getRawButton(HWR.HIGH_GEAR)){
            shiftGears(HIGH_GEAR_SERVO_VALUE, HIGH_GEAR_SERVO_VALUE);
            System.out.println("Shift Up");
            gearState = "High State";
        }
        else if(DriverStation.rightStick.getRawButton(HWR.LOW_GEAR)){
            shiftGears(LEFT_LOW_GEAR_SERVO_VALUE, RIGHT_LOW_GEAR_SERVO_VALUE);
            System.out.println("Shift Down");
            gearState = "LOW GEAR";
        }
        else if(DriverStation.rightStick.getRawButton(HWR.NEUTRAL_GEAR)){
            shiftGears(LEFT_NEUTRAL_GEAR_SERVO_VALUE, RIGHT_NEUTRAL_GEAR_SERVO_VALUE);
            System.out.println("Shift to Neutral");
            gearState = "Neutral State";
        }
//        //System.out.println(comp.getPressureSwitchValue());
//        calibrateServos();
//        
//        if(DriverStation.auxStick.getRawButton(HWR.HIGH_SHOT)&& shotHigh == false){
//            shotAngle(true);
//            shotHigh = true;
//        } else if (DriverStation.auxStick.getRawButton(HWR.LOW_SHOT)&& shotHigh == true) {
//            shotAngle(false);
//            shotHigh = false;
//        }
       

       if(DriverStation.rightStick.getTrigger() && !lastRun){
           lastRun = true;
           if (mode == RETRACT){
               mode = EXTEND;
               System.out.println("Extend arms");
           } else {
               mode = RETRACT;
               System.out.println("Retract Arms");
           }
       }
       else if (DriverStation.rightStick.getTrigger() && lastRun){
           //Do Nothing
       }
       else
       {
           lastRun = false;
       }
       extendPiston(mode);
       
       if (DriverStation.leftStick.getRawButton(HWR.ENGAGE_WINCH) && !winchIn) {
           winchIn = true;
           System.out.println("winchIn set true");

       } else if (DriverStation.leftStick.getRawButton(HWR.DISENGAGE_WINCH) && winchIn) {
           winchIn  = false;
           System.out.println("winchIn set to false");
       } 
             shiftMode(winchIn);
    }
}



