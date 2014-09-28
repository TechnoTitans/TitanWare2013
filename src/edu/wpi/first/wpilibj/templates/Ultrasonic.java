/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.AnalogChannel;
/**
 * @author Rohan Doshi
 * Converts Voltage of ultrasonic to distance
 */

public class Ultrasonic {
    public static final int ANALOG_CHANNEL = 3; // ? Correct
    
    AnalogChannel u;
    public double ultrasonicVoltage;
    public double distanceInches;
    public double distanceMilimeters;
    public double distanceMeters;
    public double distanceFeet;
    /* this class converts the analog input of our ultrasonic sensor
     * into a readeable distance in inches and milimeters
     */
    
    /**
     * gets ultrasonic voltage and limits it
     */
    public Ultrasonic(){
        u = new AnalogChannel(ANALOG_CHANNEL);
    }
    public double ultrasonicInput () {
       ultrasonicVoltage = u.getAverageVoltage();
       
       if (ultrasonicVoltage <= 0) {
           ultrasonicVoltage = 0;
       } 
       else if (ultrasonicVoltage >= 5) {
           ultrasonicVoltage = 5;
       } 
       
       //System.out.println(ultrasonicVoltage);
       return ultrasonicVoltage;
    }
    /**
     * Gets distance in Inches
     * @return distanceInches
     */
    
    public double ultrasonicInches () {
        ultrasonicInput();
        distanceInches = ultrasonicVoltage/0.009765;
        //DriverStation.logData("DISTANCE IN INCHES", distanceInches);
        return distanceInches;
    }
    /**
     * Gives distance in Feet
     * @return distanceFeet
     */
    public double ultrasonicFeet () {
        ultrasonicInches();
        distanceFeet = distanceInches/12;
        return distanceFeet;
    }
    /**
     * Gets distance in millimeters
     * Divide by 1000 to get meters
     * @return distanceMilimeters
     */
    public double ultrasonicMilimeters () {
        ultrasonicInches();
        distanceMilimeters = distanceInches*25.4;
        //DriverStation.logData("DISTANCE IN MILIMETERS", distanceMilimeters);
        return distanceMilimeters;
    }
    /**
     * Gives distance in Meters
     * @return distanceMeters
     */
    public double ultrasonicMeters () {
        ultrasonicMilimeters();
        distanceMeters = distanceMilimeters/1000;
        return distanceMeters;
    }

}

