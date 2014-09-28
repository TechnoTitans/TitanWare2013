package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Encoder;
/**
 * @author Rohan Doshi
 * 
 * This class utilizes shaft encoders to allow the robot to go an exact distance during the autonomous period. 
 */
public class DriveEncoder {
    
    private Encoder driveEncoder; // initialize field
    private int A;
    private int B;
    private int index;
    private double traveledDistance;
    private double WHEEL_DISTANCE_PER_PULSE = 0.036553894167;
    //private double WINCH_DISTANCE_PER_PULSE = 0.003974;//0.037847//03595465
    //Wheel circumfrence = 23.56 inches
    /**
     * create a new object and set equal to driveEncoder field
     * @param aChannel unused
     * @param bChannel unused
     * @param indexChannel unused
     * @param passed driveEncoder field
     * determine what parameters are needed
     */
    public DriveEncoder (int aChannel, int bChannel, boolean reverseDirection) { //determine parameters needed    
        driveEncoder = new Encoder(aChannel,bChannel, reverseDirection);
        
    }
    
    /**
     * starts the encoder
     */
    public void start() {
        driveEncoder.start();
        
    }
    
    /**
     * stops encoder
     */
    public void stop() {
        driveEncoder.stop();
    }
    
    /**
     * resets the encoder
     */
    public void reset() {
        driveEncoder.reset();
        // get()... set()...
    }
   
    public double getCount () {
        return driveEncoder.get();
    }
    /**
     * gets the last direction the encoder value changed
     * @return true or false
     */
    public boolean getDirectionForward () {
        return driveEncoder.getDirection();
        
    }
    
    /**
     * Finds distance traveled
     * @return double distance traveled in inches
     * 
     */
    public double getDistance () {
        
        driveEncoder.setDistancePerPulse(WHEEL_DISTANCE_PER_PULSE); // Need to figure out DISTANCE_PER_PULSE
        return Math.abs(driveEncoder.getDistance()); 
    }

    
    public double getDistanceFeet () {
        return getDistance()/12;
    }

    public double getDistanceMeters () {
        return getDistanceFeet() * 0.3048;
    }

            

}
    
    
    
    
    

