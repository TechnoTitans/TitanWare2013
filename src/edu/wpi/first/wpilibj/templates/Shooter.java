package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;


/**
 * This program runs the shooter motor, with two options. Button 4 is at slow 
 * speed for the five point goal during the climbing dump. Button 5 is at 
 * full speed for the three point goal while shooting from the ground. 
 * @author Rohan Doshi,Priunsh, Sarang, Michael C.
 */
    
public class Shooter {
    
    PIDJaguar jag;
    Timer shotDelay;
    Timer expelDelay;
    DigitalInput shotRPM;
    GeartoothSource tooth;
    PIDController shooterPID;
    FeedForwardLookup FFlook;
    private Solenoid sensorPower;
    private Solenoid trigger;
    private Solenoid rid;
    private final int POT_TURNS = 1;
    private static final double DEGREES_PER_ROTATION = 180.0;//for pot
    //public double PAN_ANGLE_OFFSET = 0; 
    private final double PAN_ANGLE_MID = 90.0;//theoretical mid point
    //private final double PAN_ANGLE_CENTER = PAN_ANGLE_MID + PAN_ANGLE_OFFSET;
    private final double PAN_RANGE_MAX = 15.0;//max angle from "centered"
    private final double PAN_DEADZONE_MIN = -2;
    private final double PAN_DEADZONE_MAX = 2;
    private final double RPM_ERROR_LIMIT = 200.0;
    public  double SHOT_DELAY = 0.3;
    public double aimAdjust;
    public double adjustRatio = 0;
    
    public double DIFFERENCE_LIMIT = 500;
    public double INCREMENTS = 10;
    double increment = 0;
    public double incrementDelay = 0.5;
    double lastSetpoint;
    double filterSpeed = 0.0;
    double alpha = 0.0; //must be 0 <= alpha < 1
    double clampedError = 0.0;
    
    public double setSpeed = 0;
    public double errorMargin = 0;
    
    public Shooter () {        
        jag = new PIDJaguar(HWR.SHOOTER_MOTOR);
        FFlook = new FeedForwardLookup();
        sensorPower = new Solenoid(HWR.SENSOR_POWER);
        sensorPower.set(true); //turn on power to the photogate sensor 
        jag.setExpiration(2);
        trigger = new Solenoid(HWR.SOLENOID_SLOT, HWR.SHOOT_SOLENOID);
        rid = new Solenoid(HWR.SOLENOID_SLOT, HWR.RID_SOLENOID);
        shotDelay = new Timer();
        expelDelay = new Timer();
        //FFlook = new FeedForwardLookup();
        //shotRPM = new DigitalInput(HWR.PHOTOGATE_SENSOR);
        tooth =  new GeartoothSource(HWR.PHOTOGATE_SENSOR);
        tooth.start();
        shooterPID = new PIDController(0.0 , 0.0 , 0.0, tooth, jag, 0.1);
        shooterPID.setOutputRange(-0.04, 0.04);
        //shooterPID.setTolerance(50.0); 
        
        shooterPID.enable();
        shooterPID.setSetpoint(0.0);
    }

    /**
     * Fires a weak shot at 0.3
     */
//    public void weakShot () { // while Button 4 Pressed, the Jaguar will run 
//        double weakShotSpeed = .3;
//        //DriverStation.prefDouble("WeakShotSpeed", weakShotSpeed);
//        while (DriverStation.auxStick.getRawButton(HWR.WEAK_SHOT))
//        {
//            weakShotSpeed = DriverStation.getDouble("WeakShotSpeed");
//            jag.set(weakShotSpeed); // this value can be tweaked       
//        }
        //when the button is let go, the jaguar stops
//        jag.set(0); 
//    }
//    
//    /**
//     * Fires a stronger shot at 1.0
//     */
//    public void strongShot () { // while Button 5 Pressed, the Jaguar will run 
//        while (DriverStation.auxStick.getRawButton(HWR.STRONG_SHOT)){
//            jag.set(1);
//        }
//        //when the button is let go, the jaguar stops
//        jag.set(0);
//    }
//    /**
//     * Da flip does this do...
//     * @param power 
//     */
//    public void shot (double power) { //takes power parameter from main
//        while (DriverStation.leftStick.getRawButton(HWR.STRONG_SHOT)) {
//            jag.set(power);
//        }
//    }
    /**
     * sets shooter speed from joysticks z axis
     */
    public void setShooterSpeed() {
        double speed = DriverStation.auxStick.getAxis(Joystick.AxisType.kZ);
        speed = (-speed + 1)/2;
        SmartDashboard.putNumber("Speed", speed);
        jag.set(speed);
    }
    /**
     * Sets speed from a parameter
     * @param speed 
     */
    public void setShooterSpeed(double speed) {
        jag.set(speed);
        
    }
    /**
     * actuates the piston to load frisbee
     * @param fire 
     */
    public void fire () { //needs to be implemented in TechnoTitan once piston attached.
        shotDelay.start();
        while (shotDelay.get() <= SHOT_DELAY) {
            trigger.set(true);
        }
        trigger.set(false);
        shotDelay.reset();
        shotDelay.stop();
    }
    
    public void undoMichael() { //needs to be implemented in TechnoTitan once piston attached.
        expelDelay.start();
        while (expelDelay.get() <= SHOT_DELAY) {
            rid.set(true);
        }
        rid.set(false);
        expelDelay.reset();
        expelDelay.stop();
    }
    /**
     * Turns off the shooter
     */
    public void turnOffJag () {
        jag.set(0); 
    }

//    
//    public boolean getSensorState(){
//        return shotRPM.get();
//    }
    //Pneumatics version of angle change is in Pneumatics class
    
    /**
     * function to set the roller speed.
     * 
     * @param speed Set speed in RPM
     */
    public void setFireSpeedAuto (double speed){
        if(!shooterPID.isEnable()){
            shooterPID.enable();
        }
        //SmartDashboard.putNumber("RPM", this.getRPM());
        //DriverStation.logData("L-I", shooterPID.getI());
        clampedError = clampValue(shooterPID.getError(), RPM_ERROR_LIMIT);
        SmartDashboard.putNumber("Error", clampedError);
        //DriverStation.logData("Set RPM", shooterPID.getSetpoint());
        SmartDashboard.putNumber("PID Out", shooterPID.get());
        //DriverStation.logData("Gear", tooth.);
        //shooterPID.setSetpoint(speed);
        setAlpha();//from Robot Preferences
        this.setPoint(speed);
//        if (Math.abs(lastSetpoint - speed) <= 10)
//            shooterPID.setSetpoint(speed);
//        else
//            this.setPoint(speed);
        //shooterPID.setSetpoint(speed);
        SmartDashboard.putNumber("SetRPMs", shooterPID.getSetpoint());
        SmartDashboard.putBoolean("At Speed",this.atSpeed() );
    }
//    
//    public double getError(){
//        return clampValue(shooterPID.getError(), RPM_ERROR_LIMIT);
//    }
    
    public double getError(){
        return clampValue(shooterPID.getError(), RPM_ERROR_LIMIT);
    }
    
    public double clampValue(double toClamp, double limit){
        if (toClamp > limit)
            return limit;
        else if (toClamp < -limit)
            return -limit;
        else
            return toClamp;
    }
    
    
    /**
     * Get the current RPM of the rollers
     * 
     * @return speed of rollers in RPMs
     */
    public double getRPM (){
        return tooth.pidGet();
    }
    
    public double getFF(double rpm){
        SmartDashboard.putNumber("FF", 1000*FFlook.flexInterpolator(rpm));
        return FFlook.flexInterpolator(rpm);
    }
    
    public void setPID(double P, double I, double D, double F){
        shooterPID.setPID(P, I, D, F);
    }
    
    public void setPID (double rpm) {
        double kP = DriverStation.getDouble("kP");
        double kI = DriverStation.getDouble("kI");
        double kD = DriverStation.getDouble("kD");
        double kF = this.getFF(rpm);
        shooterPID.setPID(kP, kI, kD, kF);
        //what is the feed forward term?????
    }
    
//    public void setPoint (double speed) {
//
//        double difference = speed - getRPM();
//        if (difference >= DIFFERENCE_LIMIT) {
//            increment = difference/INCREMENTS;
//        } else if (difference <= -DIFFERENCE_LIMIT) {
//            increment = difference/INCREMENTS;
//        }
//        if (difference <= -DIFFERENCE_LIMIT || difference >= DIFFERENCE_LIMIT) {
//           //lastSetpoint = shooterPID.getSetpoint();
//           shooterPID.setSetpoint(lastSetpoint+increment);
//           lastSetpoint += increment;
//           Timer.delay(incrementDelay);
//        } else {
//        shooterPID.setSetpoint(speed);
//        }
//    }
    public void setPoint(double speed){
        filterSpeed = (1.0-alpha)* speed + alpha*filterSpeed;
        setSpeed = speed;
        this.setPID(filterSpeed);
        shooterPID.setSetpoint(filterSpeed);
        SmartDashboard.putNumber("Filtered Set Point", filterSpeed);
    }
    
    public void setSetpoint(double speed){
        shooterPID.setSetpoint(speed);
    }
    
    public void setAlpha(){
        alpha = DriverStation.getDouble("Alpha");
    }
    
    public boolean atSpeed () {
        return tooth.atSpeed(setSpeed, errorMargin);
    }
    
    public void autoShoot (double speed) {
        this.setFireSpeedAuto(speed);
        if (this.atSpeed()) {
            this.fire();
            Timer.delay(0.5);
        }
    } 
        
    public double getOutput () {
        return jag.get();
    }
    
    public void powerSensor () {
        sensorPower.set(true);
    }
    public void backwards() {
        this.setFireSpeedAuto(-700);
    }
    public void unjam () {
        if (this.getRPM() <= 500) {
        shooterPID.disable();
        jag.set(-1);
        Timer.delay(0.5);
        jag.set(0);
        shooterPID.enable();
        }
    }
}
