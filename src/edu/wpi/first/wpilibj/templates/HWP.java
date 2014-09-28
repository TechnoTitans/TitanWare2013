/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

//
// file: HWPlatform.java
//
// History:
// 1/25/09 by P. Estelle - outlined possible approach as HardwareChannelConstants.h for cpp
// 1/30/10 by J. Paca    - Converted to Java as HCC.java
// 2/16/10 by P. Estelle - spilt HCC into two files.  This file defines the hardware available on
//                         the platform and will rarely change (perhaps year to year).  The other file defines
//                         how the hardware is used on a given robot implementation.
// 2/16/12 by P. Estelle - Change for FRC & cRio changes in 2012 season.  Now both 4 & 8 slot cRios are available.
//                         Usage has changed away from pyhsical slots to instances, even though the software
//                         docs still use slot terminology.  In addition, National Instruments has "crippled" the
//                         cRios in minor ways by disabling slots and requiring defined modules in defined slots
//

package edu.wpi.first.wpilibj.templates;

/**
 *  Define hardware available on cRIO platform for FRC 2009, FRC 2010, FRC 2011, FRC2012
 *
 * This is a tedious, but useful one-time-per-season task.
 * Only edit this section to add resources or correct errors
 * or when the control system changes
 *
 * The I/O available on the platform is arranged by I/O type 
 * in order to help track resource usage if I/O is scarce
 *
 * This serves as documentation and is useful to co-ordinate
 * with other project members (electrical and mechanical)
 *
 * @author MentorPete
 */
public class HWP
{

    // *** NOTE ***
    // Coding convention is that defined constants are upper case
    // not lower or mixed case.  This is to draw attention
    // to the facts that they are constants and not variables

    // Define Instances of Modules
    // at present there can be a max of two instances of any module
    public static final int  INSTANCE_1 = 1;
    public static final int  INSTANCE_2 = 2;
    
    //Define Module Instances
    //Sometimes referred to as "slots" in the wpilib software docs
    //Physical slots 1-3 are predefined and the same for both 4 and 8 slot cRio
    //Variations occur for slots 4 - 8
    //4 slot cRio has only physical slot 4 available for a second instance of any module
    //8 slot cRio has physical slots 5-7 for available for pre-defined second instances
    //8 slot cRio has physical slots 4 & 8 unavailable
    public static final int ANALOG_MODULE_1   = INSTANCE_1; //Phys 1; instance 1    
    public static final int DIGITAL_MODULE_1  = INSTANCE_1; //Phys 2; instance 1
    public static final int SOLENOID_MODULE_1 = INSTANCE_1; //Phys 3; instance 1
    public static final int ANALOG_MODULE_2   = INSTANCE_2; //Phys 4 or 5; instance 2
    public static final int DIGITAL_MODULE_2  = INSTANCE_2; //Phys 4 or 6; instance 2
    public static final int SOLENOID_MODULE_2 = INSTANCE_2; //Phys 4 or 7; instance 2
    
    // Define legacy cRIO slots on 8 slot chassis
    // and code before 2012
    public static final int  SLOT_1 = 1; //analog
    public static final int  SLOT_2 = 2; //digital
    public static final int  SLOT_3 = 3; //solenoid
    public static final int  SLOT_4 = 4; //not used
    public static final int  SLOT_5 = 5; //second analog
    public static final int  SLOT_6 = 6; //second digital
    public static final int  SLOT_7 = 7; //second solenoid
    public static final int  SLOT_8 = 8; //not used

    // Define PWM outputs
    public static final int  PWM_1 =  1;
    public static final int  PWM_2 =  2;
    public static final int  PWM_3 =  3;
    public static final int  PWM_4 =  4;
    public static final int  PWM_5 =  5;
    public static final int  PWM_6 =  6;
    public static final int  PWM_7 =  7;
    public static final int  PWM_8 =  8;
    public static final int  PWM_9 =  9;
    public static final int  PWM_10 = 10;

    //Define CAN outputs
    public static final int  CAN_1 =  1;
    public static final int  CAN_2 =  2;
    public static final int  CAN_3 =  3;
    public static final int  CAN_4 =  4;
    public static final int  CAN_5 =  5;
    public static final int  CAN_6 =  6;
    public static final int  CAN_7 =  7;
    public static final int  CAN_8 =  8;
    public static final int  CAN_9 =  9;
    public static final int  CAN_10 =  10;
    public static final int  CAN_11 =  11;
    public static final int  CAN_12 =  12;
    public static final int  CAN_13 =  13;
    public static final int  CAN_14 =  14;
    public static final int  CAN_15 =  15;
    public static final int  CAN_16 =  16;
    public static final int  CAN_17 =  17;
    public static final int  CAN_18 =  18;

    // Define GPIO (general purpose digital I/O)
    public static final int  GPIO_1 =  1;
    public static final int  GPIO_2 =  2;
    public static final int  GPIO_3 =  3;
    public static final int  GPIO_4 =  4;
    public static final int  GPIO_5 =  5;
    public static final int  GPIO_6 =  6;
    public static final int  GPIO_7 =  7;
    public static final int  GPIO_8 =  8;
    public static final int  GPIO_9 =  9;
    public static final int  GPIO_10 = 10;
    public static final int  GPIO_11 = 11;
    public static final int  GPIO_12 = 12;
    public static final int  GPIO_13 = 13;
    public static final int  GPIO_14 = 14;

    // Define Relay Outputs
    public static final int  RELAY_1 = 1;
    public static final int  RELAY_2 = 2;
    public static final int  RELAY_3 = 3;
    public static final int  RELAY_4 = 4;
    public static final int  RELAY_5 = 5;
    public static final int  RELAY_6 = 6;
    public static final int  RELAY_7 = 7;
    public static final int  RELAY_8 = 8;

    // Define Joysticks
    public static final int  JOY_1 = 1;
    public static final int  JOY_2 = 2;
    public static final int  JOY_3 = 3;
    public static final int  JOY_4 = 4;

    // Define Joystick Buttons
    public static final int  BUTTON_1 = 1; //Trigger
    public static final int  BUTTON_2 = 2;
    public static final int  BUTTON_3 = 3;
    public static final int  BUTTON_4 = 4;
    public static final int  BUTTON_5 = 5;
    public static final int  BUTTON_6 = 6;
    public static final int  BUTTON_7 = 7;
    public static final int  BUTTON_8 = 8;
    public static final int  BUTTON_9 = 9;
    public static final int  BUTTON_10 = 10;
    public static final int  BUTTON_11 = 11;

    // Define Analog
    public static final int  ANALOG_1 = 1;
    public static final int  ANALOG_2 = 2;
    public static final int  ANALOG_3 = 3;
    public static final int  ANALOG_4 = 4;
    public static final int  ANALOG_5 = 5;
    public static final int  ANALOG_6 = 6;
    public static final int  ANALOG_7 = 7;
    public static final int  ANALOG_8 = 8;

    // Define Solenoids
    public static final int  TOTAL_SOLS = 8;
    public static final int  SOLENOID_1 = 1;
    public static final int  SOLENOID_2 = 2;
    public static final int  SOLENOID_3 = 3;
    public static final int  SOLENOID_4 = 4;
    public static final int  SOLENOID_5 = 5;
    public static final int  SOLENOID_6 = 6;
    public static final int  SOLENOID_7 = 7;
    public static final int  SOLENOID_8 = 8;
    
    // Driver Station I/O (Enhanced I/O) for FRC2010
    public static final int  DSIO_1     = 1;
    public static final int  DSIO_2     = 2;
    public static final int  DSIO_3     = 3;
    public static final int  DSIO_4     = 4;
    public static final int  DSIO_5     = 5;
    public static final int  DSIO_6     = 6;
    public static final int  DSIO_7     = 7;
    public static final int  DSIO_8     = 8;
    public static final int  DSIO_9     = 9;
    public static final int  DSIO_10    = 10;
    public static final int  DSIO_11    = 11;
    public static final int  DSIO_12    = 12;
    public static final int  DSIO_13    = 13;
    public static final int  DSIO_14    = 14;
    public static final int  DSIO_15    = 15;
    public static final int  DSIO_16    = 16;

    // Define Camera & Vision
}



