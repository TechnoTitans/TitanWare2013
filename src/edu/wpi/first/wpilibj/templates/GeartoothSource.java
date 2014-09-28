/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Extend Geartooth source to use as a PID source
 * 
 * @author Ethan Everett
 * @author Mentor Pete
 */
public class GeartoothSource extends GearTooth implements PIDSource {
    //set RPM range of sensor
    private final double MAX_RPM = 4000.0; //RPM
    private final double MIN_RPM = 500.0; //RPM
    
    //compute range of periods (in sec)
    private final double MIN_PERIOD = 60.0 / MAX_RPM; //seconds
    private final double MAX_PERIOD = 60.0 / MIN_RPM; //seconds
    
    //discrete lowpass filter
    private double gearFilter = 0.0; // 0 <= value < 1.0
    private double gearOutput = 0.0;
    
    //circular buffer
    private int logIndex = 0;
    private double[] log = new double[20];
    
    private boolean atSpeed = false;
    
    /**
     * Sets a digital input as a geartooth source
     * 
     * @param digitalInput which input the sensor is using
     */
    public GeartoothSource(int digitalInput){
        super(digitalInput);
    }

    /**
     * Gets a filtered version of the geartooth sensor
     * 
     * Filter is simple digital (discrete) first order low pass type
     * Also known as an Exponentially Weighted Moving Average (EWMA)
     * 0 <= gearFilter < 1
     * 
     * @return gearOutput filtered RPM value
     */
    public double pidGet(){
        double period = super.getPeriod();
        
        //try to handle problems if we get a bad sensor reading
        if (period < MIN_PERIOD ) {
            period = MIN_PERIOD;
            System.out.println("WARN: Geartooth sensor period too short. Clamped.");
        }
        else if (period > MAX_PERIOD || Double.isInfinite(period)){
            period = MAX_PERIOD;
            System.out.println("WARN: Geartooth sensor period too long. Clamped.");       
        }
        else if (Double.isNaN(period)){
            period = 0.001;
            System.out.println("WARN: Geartooth sensor period is NaN. Bad FPGA?");
        }
                
        double rpm = 60.0 / period;
        if (rpm < MIN_RPM) rpm = MIN_RPM;
        else if (rpm > MAX_RPM) rpm = MAX_RPM;
        
        gearFilter = DriverStation.getDouble("GearFilter");
        if (gearFilter < 0.0) gearFilter = 0.0;
        else if (gearFilter >= 1.0) gearFilter = 0.9999;

        gearOutput = (1.0-gearFilter)*rpm + gearFilter*gearOutput;
        if(!Double.isNaN(gearOutput)) {
            log(gearOutput);
            return gearOutput;
        }
        else {
            System.out.println("NaN in PIDGet: " + period + " " + rpm + " " + gearFilter);
            gearOutput = 0.0;
            return gearOutput;
            //@todo: return last good value
        }
    }
    
    /**
     * Implements a circular buffer of past values
     * 
     */
    private void log(double logVal){
        log[logIndex++] = logVal;
        if(logIndex > (log.length-1)){
            logIndex = 0;//buffer wrap around
        }
    }
        
    /**
     * Determines when speed is close to the setpoint for a period of time
     * 
     * Duration depends on buffer size and sample rate
     * Example:  50 entries * 50 msec sample rate = 2.5 sec
     * 
     * @param wantSpeed is speed setpoint
     * @param errorMargin is max deviation from setpoint
     * @return atSpeed boolean true when all buffer values are less than max dev
     */
    public boolean atSpeed(double wantSpeed, double errorMargin){
        atSpeed = true;
        double maxError = errorMargin;
        for(int i=0; i<(log.length); i++){
            if (Math.abs(log[i]-wantSpeed) > maxError){
                atSpeed = false;
                break; //leave early if any outside limit (for efficiency)
            }
        }
        return atSpeed;
    }
    
    /**
     * Determines when speed is close to the setpoint for a period of time
     * 
     * Duration depends on buffer size and sample rate
     * Example:  50 entries * 50 msec sample rate = 2.5 sec
     * 
     * @param wantSpeed is speed setpoint
     * @param errorMargin is max deviation from setpoint
     * @return atSpeed boolean true when most buffer values are less than max dev
     */
    public boolean atSpeedMost(double wantSpeed, double errorMargin){
        atSpeed = true;
        double maxError = errorMargin;
        int count = (int) (log.length/10); //max number of readings out of zone
        
        for(int i=0; i<(log.length); i++){
            if (Math.abs(log[i]-wantSpeed) > maxError){
                count --;
                if (count == 0) {
                    atSpeed = false;
                    break;
                }
            }
        }
        return atSpeed;
    }
}
