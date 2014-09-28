/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 * Storage of all physical variables that connects to HWP.java
 * @author Sarang
 */
public class HWR {
    
    //Motors
    public static final int LEFT_MOTOR                  = HWP.PWM_1;
    public static final int RIGHT_MOTOR                 = HWP.PWM_2;
    //public static final int B_LEFT_MOTOR                = HWP.PWM_3;
    //public static final int B_RIGHT_MOTOR               = HWP.PWM_4;
    public static final int SHOOTER_MOTOR               = HWP.PWM_6;
    public static final int LED_VICTOR                  = HWP.PWM_5;
    public static final int SHOOTER_ANGLE_MOTOR         = HWP.PWM_7;
    
    //Joystick
    public static final int LEFT_JOYSTICK               = HWP.JOY_1;
    public static final int RIGHT_JOYSTICK              = HWP.JOY_2;
    public static final int AUX_JOYSTICK                = HWP.JOY_3;

    
    //Buttons 
    public static final int LOW_GEAR                    = HWP.BUTTON_2;
    public static final int HIGH_GEAR                   = HWP.BUTTON_3;
    public static final int NEUTRAL_GEAR                = HWP.BUTTON_4;
    public static final int GYRO_RESET                  = HWP.BUTTON_7;
    //public static final int WEAK_SHOT                   = HWP.BUTTON_4;   
    //public static final int STRONG_SHOT                 = HWP.BUTTON_5;
    //public static final int HIGH_SHOT                   = HWP.BUTTON_3;
    //public static final int LOW_SHOT                    = HWP.BUTTON_2;
    public static final int ENGAGE_WINCH                = HWP.BUTTON_3;
    public static final int DISENGAGE_WINCH             = HWP.BUTTON_2;
    public static final int SHOOT                       = HWP.BUTTON_2;
    public static final int SET_SHOOTER_SPEED           = HWP.BUTTON_3;
    public static final int AUTO_SHOOT                  = HWP.BUTTON_4;
    public static final int EXPEL_BUTTON                = HWP.BUTTON_6;
  
    
    //Solenoid
    public static final int SOLENOID_SLOT               = HWP.SOLENOID_MODULE_1;
    public static final int GEAR_SHIFT_SOLENOID         = HWP.SOLENOID_1;
    public static final int RID_SOLENOID                = HWP.SOLENOID_2;
    //public static final int LEFT_RETRACTED_PISTON       = HWP.SOLENOID_2;
    public static final int WINCH_SOLENOID              = HWP.SOLENOID_3;
    public static final int CLIMB_PISTON                = HWP.SOLENOID_4;
    public static final int SHOOT_SOLENOID              = HWP.SOLENOID_5;
    public static final int SHOOTER_ANGLE_SOLENOID      = HWP.SOLENOID_6;
    public static final int SENSOR_POWER                = HWP.SOLENOID_8; // not a solenoid. it powers a sensor. 
    
    
    //Compressors
    public static final int  COMPRESSOR_PRESSURE_SWITCH = HWP.GPIO_1;
    public static final int  COMPRESSOR_RELAY           = HWP.RELAY_8;
    
    //Sensors
    public static final int  BATTERY_VOLTAGE            = HWP.ANALOG_1;
    public static final int  BALANCE_GYRO               = HWP.ANALOG_2;
    public static final int  ULTRASONIC_SENSOR          = HWP.ANALOG_3;
    public static final int  LEFT_CHANNEL_A             = HWP.GPIO_2;
    public static final int  LEFT_CHANNEL_B             = HWP.GPIO_3;
    public static final int  RIGHT_CHANNEL_A            = HWP.GPIO_4;
    public static final int  RIGHT_CHANNEL_B            = HWP.GPIO_5;
    //Servos
    public static final int SERVO_1_SLOT                = HWP.PWM_8;
    public static final int SERVO_2_SLOT                = HWP.PWM_7;
    //Photogate
    public static final int PHOTOGATE_SENSOR            = HWP.DSIO_14;
}
