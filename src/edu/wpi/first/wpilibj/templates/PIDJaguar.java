/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;

/**
 * Extends Jaguar for a speed control loop with feedback into cRio not Jaguar
 * 
 * Adds an integrator and lowpass filtering to the input of the Jaguar.  Without
 * the integrator, the PID output is zero when at setpoint.  This is needed for
 * a velocity loop rather than a position loop.
 * Note that the jaguar can do closed loop speed control with the sensor
 * connected directly to the jaguar, but the input frequency is limited.
 * 
 * @author Ethan
 */
public class PIDJaguar extends Jaguar {
    double speedFilter = 0;
    double speedOutput = 0;
    
    public PIDJaguar(int channel){
        super(channel);
    }
    
    public void pidWrite(double output){
        output = output+get();//integrate
        if(output < 0.0) {
            output = 0.0;
        }
        
        //filter the pid controller output before writing it
        speedFilter = DriverStation.getDouble("SpeedFilter");
        speedOutput = (1.0-speedFilter)*output + speedFilter*speedOutput;
        super.pidWrite(speedOutput);
        //DriverStation.logData("PIDJag Out", output);
    }
}
