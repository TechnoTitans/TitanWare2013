/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * @author Priunsh Nagru, Rohan Doshi
 */
public class TitanGyro {
   
    //Declares basic variables for balance tracking
    public Gyro gyro; 
    private boolean isBalanced; 
    private double angle;
    private double prevAngle; 
    private final double SENSITIVITY = 0.007; //Must edit later to test sensitivity!
    //private final double SENSITIVITY = 0.0125;
    private final int DEGREE_LIMIT = 5;
   
    
    
    //Sets up 
    public TitanGyro(){
        gyro = new Gyro(HWR.BALANCE_GYRO);
        gyro.setSensitivity(SENSITIVITY);
        gyro.reset();
    }
    
    /**
     * This is a method to reset the gyro
     */   
    public void reset(){
        gyro.reset();
    }
    /**
     * Gets angle of gyro
     * @return  angle of gyro
     */
    public double angle () {
        return gyro.getAngle();
    }
    /**
     * Prints data out to SmartDashboard 
     * and RStick, button 7, resets gyro
     */
    public void run(){
        SmartDashboard.putNumber("Gyro data", gyro.getAngle());
        if(DriverStation.rightStick.getRawButton(HWR.GYRO_RESET)){
            reset();
        }
    }
    
        
            
        
           
}
        
    
    
   
        
        
 
