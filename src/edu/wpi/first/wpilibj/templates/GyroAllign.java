package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.templates.GearTrain;
//import edu.wpi.first.wpilibj.templates.TitanGyro;
/**
 * The purpose of this program is to compare the robot's ability to go in a 
 * straight line with and without the gyro input. 
 * @author Rohan Doshi
 */
public class GyroAllign {
    //GearTrain driveForward;
    TitanGyro gyroAllignVar; 
    public double LEFT_MOTOR_SPEED_DEFAULT  = .35;
    public double RIGHT_MOTOR_SPEED_DEFAULT = .35; 
    double leftMotorS = .5;
    double rightMotorS = .5;
    
    public double rightSpeed;
    public double leftSpeed;
    
    public GyroAllign(TitanGyro passed) {
        //riveForward = new GearTrain();
        gyroAllignVar = passed;
    }
    
    /**
     * drives with no gyro assistance at speed .3 for both motors
     */
    public void noGyroForward () {
        double leftMotorS = .3;
        double rightMotorS = .3; 
        //driveForward.setMotorSpeed(leftMotorS,rightMotorS);
    }
    /**
     * Drives forward using gyro assistance
     * threshold of 2 degrees
     */
    public void withGyroForward () {


        double INCREASE_FACTOR = 1.25;
        double angleLimit = 2; //lower threshold 
        
        //go forward while adjusting with gyro
            //driveForward.setMotorSpeed(LEFT_MOTOR_SPEED_DEFAULT,RIGHT_MOTOR_SPEED_DEFAULT);
            
            double gyroAngle = gyroAllignVar.angle();
            if (gyroAngle > angleLimit) {
                //rightMotorS = rightMotorS * INCREASE_FACTOR; 
                //driveForward.setMotorSpeed(LEFT_MOTOR_SPEED_DEFAULT,rightMotorS);  
                rightSpeed = rightMotorS;
                leftSpeed = LEFT_MOTOR_SPEED_DEFAULT;
                //Timer.delay(0.2);
                
            }
            else if (gyroAngle < -angleLimit) {
                //leftMotorS = leftMotorS * INCREASE_FACTOR;
                //driveForward.setMotorSpeed(leftMotorS, RIGHT_MOTOR_SPEED_DEFAULT);
                leftSpeed = leftMotorS;
                rightSpeed = RIGHT_MOTOR_SPEED_DEFAULT;
                //Timer.delay(0.2);
            }
    }
    /**
     * gets speed of right motor
     * @return speed of right motor
     */
    public double getRightSpeed () {
        return rightSpeed;
    }
    /**
     * gets speed of left motor
     * @return speed of left motor
     */
    public double getLeftSpeed () {
        return leftSpeed;
    }
    /**
     * resets the gyro
     */
    public void gyroReset () {
        gyroAllignVar.reset();
        Timer.delay(.5);    
    }
}
