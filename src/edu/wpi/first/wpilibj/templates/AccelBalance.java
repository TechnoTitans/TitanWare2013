/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.I2C;


/**
 *This class is used for balancing the robot while climbing
 * @author Priunsh Nagru
 */
public class AccelBalance {
    public ADXL345_I2C accel;
    public I2C bus;
    private double xAxis;
    private double yAxis;
    private double zAxis;
    private double xZAngle;
    private double yZAngle;
    private final double INCLINE_ANGLE = 45;
    private final double INCLINE_THRESHOLD = 5;
    private double[] rawAxisTable = new double[3];
    private double[] normalAxisTable = new double[3];
    private int host = 0x2D; 
    private int command = 0x08;
    
    
    
    
    //initializes class by creating an ADXL345 object
    public AccelBalance(){
        accel = new ADXL345_I2C(HWP.DIGITAL_MODULE_1, ADXL345_I2C.DataFormat_Range.k2G);
        //bus = new I2C(HWP.)
        //accel.write(host, command);
        
    }
    
    /**
     * Used to get the x axis g-force
     * @return double value for the x axis
     */
    
    public double getXAxis(){
        return accel.getAcceleration(ADXL345_I2C.Axes.kX);
    }
    
    /**
     * Used to get the y axis g-force
     * @return double value for the y axis
     */
    public double getYAxis(){
        return accel.getAcceleration(ADXL345_I2C.Axes.kY);
    }
    
    /**
     * Used to get the z axis g-force
     * @return double value for the z axis
     */
    public double getZAxis(){
        return accel.getAcceleration(ADXL345_I2C.Axes.kZ);
    }
    
    /**
     * Updates the raw values in the table by using get methods 
     */
    public void updateRawTable(){
        rawAxisTable[0] = getXAxis();
        rawAxisTable[1] = getYAxis();
        rawAxisTable[2] = getZAxis();   
    }  
    
    /**
     * Normalizes and updates the table to hold unit values for the axes
     */
    public void updateNormalTable(){
        updateRawTable();
        double x = rawAxisTable[0];
        double y = rawAxisTable[1];
        double z = rawAxisTable[2];
        double length; 
        length = Math.sqrt((x*x)+(y*y)+(z*z));
        x = x / length; 
        y = y / length;
        z = z / length;
        normalAxisTable[0] = x;
        normalAxisTable[1] = y;
        normalAxisTable[2] = z;
    }
    
    /**
     * Method returns angle in degrees on the x-plane
     * @return angle in degrees on the x-plane relative to z
     */
    public double getXZAngle(){
        double xUnit = normalAxisTable[0];
        double zUnit = normalAxisTable[2];
        return Math.toDegrees(MathUtils.atan2(xUnit,zUnit));
    }
        
   /**
    * Method returns angle in degrees on the y-plane
    * @return angle in degrees on the y-plane relative to z
    */
    public double getYZAngle(){
        double yUnit = normalAxisTable[1];
        double zUnit = normalAxisTable[2];
        return Math.toDegrees(MathUtils.atan2(yUnit,zUnit));
    }
    /** Y 
     *  ^
     *  |
     *  |
     *  Z-----> X
     * @return true if the robot is in the correct angle to climb
     */
    public boolean checkInclineAngle(){
        double inclineAngle = getYZAngle();
        if(Math.abs(INCLINE_ANGLE - inclineAngle) < INCLINE_THRESHOLD){
            return true;
        }
        else{
            return false;
        }
    }
    
    /**
     * Sends all data from LED to driver station and computer
     */
    public void run(){
//        System.out.println("Raw X " + getXAxis());
//        System.out.println("Raw Y " + getYAxis());
//        System.out.println("Raw Z " + getZAxis());
        updateNormalTable();
        DriverStation.sendData("Raw X", getXAxis());
        DriverStation.sendData("Raw Y", getYAxis());
        DriverStation.sendData("Raw Z", getZAxis());
        System.out.println("X-Z Angle: " +getXZAngle());
        System.out.println("Y-Z Angle: " +getYZAngle());
        DriverStation.sendData("X-Z Angle", getXZAngle());
        DriverStation.sendData("Y-Z Angle", getYZAngle());
    }
}

