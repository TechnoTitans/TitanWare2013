/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the brightness/intensity of the LED rings used by the camera
 * 
 * The LEDs are connected to a "Victor" speed control for variable intensity.
 * They are inside a bridge rectifier for reverse polarity protection.
 *
 * @author Priunsh Nagru
 */
public class LEDIndicator {
    Victor lightControl;
    double percentBrightness;
    Timer time;
    private double prevTime;
    
    private final double OFF_VALUE = 0.15; //off value for victor hardware driving LED ring
    private final double INITIAL_INTENSITY  = 50.0; //percent (arbitrary)
    private final double INTENSITY_OFFSET = 20.0; //intensity at distance=0 in percent
    private final double INTENSITY_SLOPE = 4.0; //percent per foot
    
    private final double SINE_DELAY = 0.075;
    private final double LINEAR_INCREMENT = 1;
    
    private double d = 30.0; //initial distance (ft)
    
    /**
     * Constructor
     * 
     */
    public LEDIndicator(){
        lightControl = new Victor(HWR.LED_VICTOR);
        percentBrightness = INITIAL_INTENSITY;
        time = new Timer();
    }
    /**
     * Sets the LED to full brightness
     */
    public void ledOn(){
        setBrightness(100.0);
    }
    /**
     * Sets the LED off
     * 
     */
    public void ledOff(){
        setBrightness(0.0);
    }
    
    /**
     * Takes in a percent and converts it to a PWM value the controller can use
     * 
     * Also updates percent brightness attribute
     * @param percentBright
     * @return PWMvalue from OFF_VALUE to 1.0
     */
    public double convertPercent(double percentBright){
        if (percentBright < 0.0){
           percentBrightness = 0.0;
           return OFF_VALUE;
        }
        else if (percentBright > 100.0){
           percentBrightness = 100.0;
           return 1.0;
        }
        else {
           percentBrightness = percentBright;
           return (1.0 - OFF_VALUE) * (percentBright / 100.0) + OFF_VALUE;
        }
    }
    
    /**
     * Actually sets the brightness via the victor
     * @param percentBright 
     */
    public void setBrightness(double percentBright){
        lightControl.set(convertPercent(percentBright));
        //System.out.println("Setting brightness");
    }
    
    /**
     * Used to retrieve current brightness
     * @return percentBrightness 
     */
    public double getBrightness(){
        return percentBrightness;
    }
        
    /**
     * Changes the brightness as a function of distance
     * @param distance in feet (positive, >=0)
     */
    public void changeBrightness(double distance){
        double percentBright;
        if (distance < 0.0) distance = 0.0; //
        percentBright = INTENSITY_SLOPE * distance + INTENSITY_OFFSET;
        setBrightness(percentBright);
    }
    
    /** FIXME
     * Increments the brightness level by a given value
     * @param byPercentBright 
     */
    public void incrementBrightness (double byPercentBright){
        if (byPercentBright + percentBrightness <= 100 && byPercentBright + percentBrightness >= 0 ){
           percentBrightness += byPercentBright; 
           setBrightness(percentBrightness);
        }
    }
    
    /** FIXME
     * Decrements the brightness level by a given value
     * @param byPercentBright 
     */        
    public void decrementBrightness (double byPercentBright){
        if (byPercentBright - percentBrightness <= 100 && byPercentBright - percentBrightness >= 0 ){
           percentBrightness -= byPercentBright; 
           setBrightness(percentBrightness);
        } 
    }
        
    /** FIXME
     * Turns the LED off and then puts it through one cycle of a sine wave
     */
    public void sinePulse(int numPulses){
        ledOff();
        for(int x = 0; x <= numPulses; x++){
            for(int i = 0; i <= 180; i++){
                setBrightness(100 * Math.abs(Math.sin(i)));
            }
        }
    }
    
    /** FIXME
     * Puts the LED through a linear pulse
     */
    public void linearPulse(int numPulses){
        ledOff();
        for(int x = 0; x <= numPulses; x++){
            time.start();
            prevTime = time.get();
            for(double i = getBrightness(); time.get() - prevTime >= 30 && getBrightness() <= 100; i++){
                incrementBrightness(5);
                prevTime = time.get();
            }
            prevTime = time.get();
            for(double i = getBrightness(); time.get() - prevTime >= 30 && getBrightness() >= 0; i--){
                decrementBrightness(5);
                prevTime = time.get(); 
            }
        }
    }
    
    //FIXME
    public void linearCliffPulse(int numPulses){
        ledOff();
        for(int x = 0; x <= numPulses; x++){
            time.start();
            prevTime = time.get();
            for(double i = getBrightness(); time.get() - prevTime >= 0.5 && getBrightness() <= 100; i++){
                incrementBrightness(1);
                prevTime = time.get();
            }
        }
    }
    
    //FIXME
    public void sineCliffPulse(int numPulses){
        ledOff();
        for(int x = 0; x <= numPulses; x++){
            for(int i = 0; i <= 90; i++){
                setBrightness(100 * Math.abs(Math.sin(i)));
            }
            ledOff();
        }
    }
    
    //FIXME
    public void sineLinearPulse(int numPulses){
        ledOff();
        for(int x = 0; x <= numPulses; x++){
            for(int i = 0; i <= 90; i++){
                setBrightness(100 * Math.abs(Math.sin(i)));
            } 
            for(int i = 100; i >= 0; i--){
                setBrightness(i); 
            }
        }
    }
 
    /**
     * periodic method for light control
     * 
     * typically called in autonomous or teleop every 20 msec or so (50 Hz)
     * therefore it's important that this be "non-blocking"
     * 
     */
    public void run(){
//        incrementBrightness(10);
        //setBrightness(DriverStation.getDouble("PctBrightness"));
        //setBrightness(75);
        //DriverStation.prefDouble("Distance", 0.0);
        //double d = DriverStation.getDouble("Distance");
        
        //simulate driving towards the target
        //changeBrightness(d);
        //d -= 0.075; //take about 8 seconds to reach target (distance=0)
        setBrightness(DriverStation.getDouble("PctBrightness"));
//        Timer.delay(0.5);
        SmartDashboard.putNumber("PctBrightness", getBrightness());
        //linearPulse(2);
//        System.out.println(getBrightness());
    }
}