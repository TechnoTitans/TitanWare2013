/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import java.lang.Math.*;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class will be used to calculate and get needed information based on vision tracking.
 * @author Michael
 */
public class VisionTracking {
    //Bottom target dimentions: 29"x24"
    //with tape: 37"x32"
    //cool fact: when two feet away, a one foot ruler is 3/5 of the screen width and 4/5 of the screen height
    //Camera IP: 10.16.83.11
//    private final double PIXEL_COUNT_X = 320;
    private final double PIXEL_COUNT_Y = 240;
//    private final double CAMERA_FOV_HORIZONTAL_ANGLE = 22.620; //in degrees
    private final double CAMERA_FOV_VERTICAL_ANGLE = 17.354;
//    private final double CAMERA_MOUNT_ANGLE = 25;
//    private final double LOW_TARGET_WIDTH = 37; //in inches
//    private final double LOW_TARGET_HEIGHT = 32;
    private final double HIGH_TARGET_WIDTH = 62;
    private final double HIGH_TARGET_HEIGHT = 20;
//    private final double ROBOT_LENGTH = 37.0/12.0;//in feet, distance from camera to front edge of robot
    private final double HIGH_TARGET_VERTICAL_DISTANCE = 78; //in inches from camera to bottom of target
    //high target bottom is 104.125 inches off ground
    private double pixPseudoWidth;
    private double pixPseudoHeight;
//    private double widthProp,screenWidth;
    private double heightProp,screenHeight;
    private double horizontalDistanceInches,distanceInches,distanceFeet,distanceMeters;
    private double pixWidth,pixHeight,targetAngleRad,targetAngleDeg;
    private double pixTopWidth,pixBottomWidth,pixRightHeight,pixLeftHeight;
    private double theoCameraMountAngle;
    private double pseudoDistanceInches;
    private boolean angleDirection = true;

     /**
     * Sets dimensions of blob to variables
     * @param width the width in pixels of the target blob
     * @param height the height in pixels of the target blob
     */
    public void setTargetDimensions(double width,double height){
        pixPseudoWidth = width;
        pixPseudoHeight = height;
    }
    
    /**
     * Sets dimensions of blob to variables
     * @param topWidth the width of top of target blob in pixels
     * @param bottomWidth the width of bottom of target blob in pixels
     * @param rightHeight the height of right of target blob in pixels
     * @param leftHeight the height of left of target blob in pixels
     */
    public void setTargetDimensions(double topWidth,double bottomWidth,double rightHeight,double leftHeight){
        pixTopWidth = topWidth;
        pixBottomWidth = bottomWidth;
        pixRightHeight = rightHeight;
        pixLeftHeight = leftHeight;
        pixPseudoWidth = (pixTopWidth+pixBottomWidth)/2;
        pixPseudoHeight = (pixRightHeight+pixLeftHeight)/2;
    }
    
    /**
     * Calculates distance from target
     */
    public void calcTargetDist(){
//        pixHeight = pixPseudoHeight/Math.cos(CAMERA_MOUNT_ANGLE);
//        widthProp = pixWidth/PIXEL_COUNT_X;
//        heightProp = pixHeight/PIXEL_COUNT_Y;
//        screenWidth = LOW_TARGET_WIDTH/widthProp;
//        screenWidth = HIGH_TARGET_WIDTH/widthProp;
//        distanceInches = screenWidth/(2*Math.tan(CAMERA_FOV_HORIZONTAL_ANGLE));
//        screenHeight = LOW_TARGET_HEIGHT/heightProp;
//        screenHeight = HIGH_TARGET_HEIGHT/heightProp;
//        distanceInches = screenHeight/(2*Math.tan((CAMERA_FOV_VERTICAL_ANGLE)*Math.PI/180));
//        horizontalDistanceInches = Math.sqrt(distanceInches*distanceInches-HIGH_TARGET_VERTICAL_DISTANCE*HIGH_TARGET_VERTICAL_DISTANCE);
//        distanceFeet = horizontalDistanceInches/12;
//        distanceFeet = distanceInches/12;
//        distanceMeters = distanceInches*0.0254;
//        distanceFeet -= ROBOT_LENGTH;
        pseudoDistanceInches = Math.sqrt((HIGH_TARGET_HEIGHT*HIGH_TARGET_VERTICAL_DISTANCE)/((pixBottomWidth/pixTopWidth)-1));
        theoCameraMountAngle = MathUtils.asin(HIGH_TARGET_VERTICAL_DISTANCE/pseudoDistanceInches);
        pixHeight = pixPseudoHeight/Math.cos(theoCameraMountAngle);
        heightProp = pixHeight/PIXEL_COUNT_Y;
        screenHeight = HIGH_TARGET_HEIGHT/heightProp;
        distanceInches = screenHeight/(2*Math.tan((CAMERA_FOV_VERTICAL_ANGLE)*Math.PI/180));
        horizontalDistanceInches = Math.sqrt(distanceInches*distanceInches-HIGH_TARGET_VERTICAL_DISTANCE*HIGH_TARGET_VERTICAL_DISTANCE);
        distanceFeet = horizontalDistanceInches/12;
    }
    
    /**
     * Sees if robot is left or right of target. If angleDirection is true, robot is on the right. If angleDirection is false, robot is on the left.
     */
    public void angleDirection(){
        if(pixRightHeight>pixLeftHeight){
            angleDirection = true;
        }
        else{
            angleDirection = false;
        }
    }
    
    /**
     * Calculates angle target is turned away from the camera
     */
    public void calcTargetAngle(){
        pixWidth = pixHeight*(HIGH_TARGET_WIDTH/HIGH_TARGET_HEIGHT);
//        pixWidth = pixHeight*(LOW_TARGET_WIDTH/LOW_TARGET_HEIGHT);
        if (pixPseudoWidth>pixWidth){
            targetAngleRad = 0;
            targetAngleDeg = 0;
        }
        else{
        targetAngleRad = (Math.PI/2)-MathUtils.asin(pixPseudoWidth/pixWidth);
        targetAngleDeg = targetAngleRad*180/Math.PI;
        }
        if (targetAngleDeg == Double.NaN){
            targetAngleDeg = 0;
            System.out.println("Angle of NaN");
        }
        if (angleDirection == false){
            targetAngleDeg *= -1;
        }
    }
    
    /**
     * Returns distance in inches
     * @return target distance in inches
     */
    public double getTargetDistInches(){
        return horizontalDistanceInches;
    }
    
    /**
     * Returns distance in feet
     * @return target distance in feet
     */
    public double getTargetDistFeet(){
        return distanceFeet;
    }
    
    /**
     * Returns distance in meters
     * @return target distance in meters
     */
    public double getTargetDistMeters(){
        return distanceMeters;
    }
    
    /**
     * Returns angle target is turned away from the camera
     * @return target angle in degrees
     */
    public double getTargetAngle(){
        return targetAngleDeg;
    }
    
    /**
     * Calculates distance from target and returns it
     * @param width width of target in pixels
     * @param height height of target in pixels
     * @return distance from target in feet
     */
    public double run(double width,double height){
        setTargetDimensions(width,height);
        calcTargetDist();
        return distanceFeet;
    }
    
    /**
     * Calculates distance from target and target angle and returns distance
     * @param topWidth the width of top of target blob in pixels
     * @param bottomWidth the width of bottom of target blob in pixels
     * @param rightHeight the height of right of target blob in pixels
     * @param leftHeight the height of left of target blob in pixels
     * @return distance from target in feet
     */
    public double run(double topWidth,double bottomWidth,double rightHeight,double leftHeight){
        setTargetDimensions(topWidth,bottomWidth,rightHeight,leftHeight);
        angleDirection();
        calcTargetDist();
        calcTargetAngle();
        return distanceFeet;
    }
    
}
