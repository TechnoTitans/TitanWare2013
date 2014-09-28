/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * @author Rohan Doshi, Priunsh Nagru 2013
 * This class designates the inputs from the controller.
 */
public class DriverStation {
    /**
     * Axis : X
     */
    public static final int kXAxis = 1;
    /**
     * Axis : Y
     */
    public static final int kYAxis = 2;
    /**
     * Axis : Z
     */
    public static final int kZAxis = 3;
    
    public static Joystick leftStick = new Joystick(HWR.LEFT_JOYSTICK);
    public static Joystick rightStick = new Joystick(HWR.RIGHT_JOYSTICK);
    public static Joystick auxStick = new Joystick(HWR.AUX_JOYSTICK);
    
    public static boolean[][] lasts = new boolean[3][11];
    
    
   
    
    public static void reset(){
        for (int i=0;i<3;i++)
            for(int j=0;j<11;j++)
                lasts[i][j] = false;
    }
     /*
     * Methods to send data to SmartDashboard
     */
        
    /**
     * Sends the value to SmartDashboard 
     * @param key Value name
     * @param val double Input value
     */
    public static void sendData(String key, double val){
        try{
            SmartDashboard.putNumber(key,val);
        }
        catch(IllegalArgumentException e){
        }
    }
    
    /**
     * Sends the value to SmartDashboard
     * @param key Value name 
     * @param val int Input value
     */
    public static void sendData(String key, int val){
        try{
            SmartDashboard.putNumber(key, val);
        }
        catch(IllegalArgumentException e){
        }
    }
     
    /**
     * Sends the value to SmartDashboard
     * @param key Value name 
     * @param val String Input value
     */
    public static void sendData(String key, String val){
        try{
            SmartDashboard.putString(key, val);
        }
        catch(IllegalArgumentException e){
        }
    }
    
    /**
     * Sends the value to SmartDashboard
     * @param key Value name 
     * @param val boolean Input value
     */
    public static void sendData(String key, boolean val){
        try{
            SmartDashboard.putBoolean(key, val);
        }
        catch(IllegalArgumentException e){
        }
    }
    
    /*
     * Methods to send data to store in Preferences 
     */
    
    /**
     * Sends the integer to be stored in Preferences in the cRIO
     * @param key Name of the value 
     * @param val int Value to be sent 
     */
    public static void prefInt(String key, int val){
        Preferences.getInstance().putInt(key, val);
    }
    
    /**
     * Sends the boolean to be stored in Preferences in the cRIO
     * @param key Name of the value 
     * @param val boolean Value to be sent 
     */
    public static void prefBoolean(String key, boolean val){
        Preferences.getInstance().putBoolean(key, val);
    }
    
    /**
     * Sends the String to be stored in Preferences in the cRIO
     * @param key Name of the value
     * @param val String Value to be sent 
     */
    public static void prefString(String key, String val){
        Preferences.getInstance().putString(key, val);
    }
    
    /**
     * Sends the double to be stored in Preferences in the cRIO
     * @param key Name of the value
     * @param val double value to be sent
     */
    public static void prefDouble(String key, double val){
        Preferences.getInstance().putDouble(key, val);
    }
    
    /*
     * Methods to get data from the cRIO
     */
    
     /**
     * Receives the value from cRIO
     * Default is false if no boolean value is found with the key
     * @param key Value name 
     */
    public static void getBoolean(String key){
        Preferences.getInstance().getBoolean(key, false);
    }
    
    /**
     * Receives the value from cRIO
     * Default is "null" if no string is found with the key
     * @param key Value name 
     * @return null if no string is found
     */
    public static String getString(String key){
        return Preferences.getInstance().getString(key, null);
    }
    
    /**
     * Receives the value from cRIO
     * Default is 0.0 if no double is found with the key
     * @param key Value name 
     * @return 0.0 if key is not found
     */
    public static double getDouble(String key){
        return Preferences.getInstance().getDouble(key, 0.0);
    }
    
    /**
     * Receives the value from cRIO
     * Default is 0 if no integer found with the key
     * @param key Value name 
     * @return 0 if key is not found 
     */
    public static int getInt(String key){
       return Preferences.getInstance().getInt(key, 0);
    }
    
    /**
     * 
     */
    public static boolean antiBounce(int joystick, int button){
       boolean pressed = false;
       if (joystick == 1)
           pressed = DriverStation.leftStick.getRawButton(button);
       else if (joystick == 2)
           pressed = DriverStation.rightStick.getRawButton(button);
       else if (joystick == 3)
           pressed = DriverStation.auxStick.getRawButton(button);
       
       if(pressed && !lasts[joystick-1][button-1]){
           lasts[joystick-1][button-1] = true;
           return true;
       }
       else if (pressed && lasts[joystick-1][button-1]){
           return false;
       }
       else
       {
           lasts[joystick-1][button-1] = false;
           return false;
       }
    }
}
    
   
   
