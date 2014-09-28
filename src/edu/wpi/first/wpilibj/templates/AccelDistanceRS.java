package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This program uses the accelerometer to calculate elapsed distance using kinematics. 
 * Output is sent to the Smart Dashboard.
 * @author Rohan Doshi
 */
public class AccelDistanceRS {
    private ADXL345_I2C accelX;
    private double accelValue;
    private Timer accelTime = new Timer();
    private double totalDistance = 0;
    private double additionalDistance = 0;
    private double elapsedTimeSoFar = 0;
    private double oldTime = 0;
    private double deltaTime = 0;
  
    
    public AccelDistanceRS () { //constructor
        accelX = new ADXL345_I2C(HWP.DIGITAL_MODULE_1, ADXL345_I2C.DataFormat_Range.k2G); 
        resetDistance();
    }
    public double getXAxis(){ //unused
      
        return accelX.getAcceleration(ADXL345_I2C.Axes.kX);
    }
    public double getAdditionalDistance () {
        elapsedTimeSoFar = accelTime.get();
        deltaTime = elapsedTimeSoFar - oldTime;
        accelValue = accelX.getAcceleration(ADXL345_I2C.Axes.kX);
        SmartDashboard.putNumber("Acceleration", accelValue);
        System.out.println("Acceleration is " + accelValue);
        additionalDistance = .5 * accelValue * 9.81 * deltaTime * deltaTime * 39.71; // 1) kinematics: x = (1/2) * a * t^2 2) 1 g acceleration = 9.81 m/s^2 
        oldTime = elapsedTimeSoFar;
        return additionalDistance;     
    }
    public double updateDistance () { 
        //updates total distance in Smart Dashboard and can return the value if necessary
        totalDistance = totalDistance + getAdditionalDistance(); 
        
        SmartDashboard.putNumber("Total Distance", totalDistance);
        System.out.println("Total distance is: " + totalDistance);
        return totalDistance;
    }
    public void resetDistance () {
        totalDistance = 0;
        accelTime.reset(); 
        accelTime.start();
    }
  
} //end program
    
/* INSTRUCTIONS
* to get total distance in main and output in Smart Dashboard:
* 
*       AccelDistanceRS distanceTracker;
*       distanceTracker = newAccelDistanceRS
*
*       autonomous loop {
*           distanceTracker.updateDistance();
*       } 
*/
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
// ALTERNATIVE VERSION - uses inputed distance to reach specific distance
//    public boolean hitTargetDistance (int targetDistance) { //uses x axis of accelerometer - check
//        boolean hitTarget = false;
//        double deltaTime = .5; 
//        double totalDistanceElapsed = 0;
//        double additionalDistance = 0;
//
//        while (hitTarget = false){ 
//            // check if hit target distance
//            if (totalDistanceElapsed > targetDistance) {
//                hitTarget = true;
//            }
//            //add additional distance elapsed - right Reimann sum
//            Timer.delay(deltaTime);
//            accelValue = accelX.getAcceleration(ADXL345_I2C.Axes.kX);
//            additionalDistance = accelValue * deltaTime * deltaTime * .5; // kinematics: x = (1/2) * a * t^2
//            totalDistanceElapsed = totalDistanceElapsed + additionalDistance;
//        }   
//        return true;
//        
//        /*main program would say something like:
//         *      something.setMotorSpeed (leftMotorSpeed, rightMotorSpeed);
//                while (object.hitTargetDistance( ur target distance ) = false){
//        *       }
//        *       something.setMotorSpeed (0,0);
//        * 
//        */
//    }
    
    /////////////////////////////////////////////////////////////////////////


