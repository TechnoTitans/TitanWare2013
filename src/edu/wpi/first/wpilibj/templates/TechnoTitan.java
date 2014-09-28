/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class TechnoTitan extends IterativeRobot {

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    //Jaguar rightMotor;
    //Jaguar leftMotor;
    DriverStation control;
    NetworkTable table;
    GearTrain drive;
    Pneumatics air;
    TitanGyro gyro;
    Shooter weinberger; //Reference
    GyroAllign gyroDrive;
    Timer time;
    Timer autoClock;
    LEDIndicator priunsh;
    DriveEncoder leftEncoder;
    DriveEncoder rightEncoder;
    VisionTracking track;
    double targetRightHeight, targetLeftHeight;
    double targetTopWidth, targetBottomWidth;
    double targetHeight;
    double targetWidth;
    double targetDistFeet;
    double targetAngleDeg;
    double currentTimeElapsed = 0.0; //set timer to zero
    boolean last = false;
    
    double AUTO_SHOT_RPM = 3000.0;//WILL FINALIZE AFTER TESTING
    final double AUTO_ERROR_LIMIT = 100.0;
    final double TELEOP_ERROR_LIMIT = 75;
    final double INIT_REV_SPEED = 1500.0;
    final double SHOTS_LOADED = 3;
    final double IDLE_RPM = 1800.0;
    final double SHOT_DELAY = 0.5;
    final double AUTO_TURN = 30;
    final double AUTO_END_TURN = 130 + AUTO_TURN;
    final double AUTO_HALF_TURN = 180;
    double AUTO_DRIVE_SPEED = 0.20;
    double AUTO_TURN_SPEED = 0.4;
    final double AUTO_DRIVE_TIME = .6;
    final int LEFT_SIDE = 1;
    final int CENTER = 2;
    final int CENTER_V2 = 4;
    final int RIGHT_SIDE = 3;
    final double P = 0.000035;
    final double I = 0.0000010;
    final double D = 0.00060;
    final double F = 0.0000023;
        
                
    int targetCenter = 160;
    double targetTolerance = .05;
    double adjustmentScale = 1.5;
    double drivingSpeed = 0.35; //current 0.35 is good
    int stoppingDistance = 4;
    double turningSpeed = 0.4; // currently 0.4 is good
    double TIME_CONSTANT = 1.5;
    double encoderDist = 1;
    int mode;
    int shotsFired;
    boolean firing;
    double autoMode;
    boolean camExposure;
    final double RIGHT_DRIVE_DISTANCE = 5.125; //feet
    final double RIGHT_TURN = 27; //degrees, turn when you are on the right side
    double CENTER_DRIVE_DISTANCE = 3.083333; //feet - This is 37 inches
    double DRIVE_BACK_DISTANCE = 8;
    double SIDE_DRIVE_BACK_DISTANCE = 4;
    final double LEFT_DRIVE_DISTANCE = 5.125; //feet
    final double LEFT_TURN = -27; // degrees , turn when you are on the left side
    final double AUTO_SPEED = 0.35;
    double distanceDriven;
    double angleTurned;
    double autoShotSpeed;

    public void robotInit() {
        //Pneumatics Init
        air = new Pneumatics();
        air.gearState = "High Gear";
        air.shiftGears(air.HIGH_GEAR_SERVO_VALUE, air.HIGH_GEAR_SERVO_VALUE);
        air.shiftMode(false);
        air.startCompressor();

        //Drive/Gear Train Init
        drive = new GearTrain();
        gyro = new TitanGyro();
        gyroDrive = new GyroAllign(gyro);
        priunsh = new LEDIndicator();
        //LiveWindow.setEnabled(true);

        leftEncoder = new DriveEncoder(HWR.LEFT_CHANNEL_A, HWR.LEFT_CHANNEL_B, true);
        rightEncoder = new DriveEncoder(HWR.RIGHT_CHANNEL_A, HWR.RIGHT_CHANNEL_B, true);
        DriverStation.prefDouble("High", air.HIGH_GEAR_SERVO_VALUE);
        DriverStation.prefDouble("LeftLow", air.LEFT_LOW_GEAR_SERVO_VALUE);
        DriverStation.prefDouble("RightLow", air.RIGHT_LOW_GEAR_SERVO_VALUE);
        DriverStation.prefDouble("LeftN", air.LEFT_NEUTRAL_GEAR_SERVO_VALUE);
        DriverStation.prefDouble("RightN", air.RIGHT_NEUTRAL_GEAR_SERVO_VALUE);


        //Shooter/Vision Init
        track = new VisionTracking();
        weinberger = new Shooter();
        weinberger.setFireSpeedAuto(0);

    }
    public void disablePeriodic () {
        weinberger.setFireSpeedAuto(0);
    }
    /**
     * This method is called at the beginning of the autonomous period
     */
    public void autonomousInit () {
        AUTO_DRIVE_SPEED = DriverStation.getDouble("AutoDrive");
        AUTO_TURN_SPEED = DriverStation.getDouble("AutoTurn");
        autoMode = DriverStation.getInt("AutoMode");
        
        time = new Timer();
        autoClock = new Timer();
        autoClock.reset();
        autoClock.start();
        time.reset();
        time.start();
        currentTimeElapsed = 0.0;
        gyro.reset();
        priunsh.setBrightness(70);
        air.startCompressor();
        //weinberger.setPID();
        //weinberger.setSetpoint(INIT_REV_SPEED);
        weinberger.setFireSpeedAuto(INIT_REV_SPEED);
        SmartDashboard.putNumber("Brightness", priunsh.getBrightness());


        //Shooter Set up
        //weinberger.setPID();
        weinberger.errorMargin = AUTO_ERROR_LIMIT;
        weinberger.filterSpeed = 0.0;
        AUTO_SHOT_RPM = DriverStation.getDouble("AutoRPM");
        weinberger.setFireSpeedAuto(0); //or AUTO_SHOT_RPM
        autoShotSpeed = 0;
        mode = 0;
        shotsFired = 0;
        firing = false;

        weinberger.errorMargin = AUTO_ERROR_LIMIT;
        weinberger.powerSensor();
        weinberger.filterSpeed = 0.0;
        //Autonomous driving set up
        air.shiftGears(air.LEFT_LOW_GEAR_SERVO_VALUE, air.RIGHT_LOW_GEAR_SERVO_VALUE);
        leftEncoder.start();
        rightEncoder.start();
        leftEncoder.reset();
        rightEncoder.reset();
        gyro.reset();
        distanceDriven = 0;
        CENTER_DRIVE_DISTANCE = DriverStation.getDouble("AutoDist");
        angleTurned = 0;
        drive.setMotorSpeed(0.0, 0.0);

    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        //NEW AUTONOMOUS CODE
        SmartDashboard.putNumber("RPM", weinberger.getRPM());
        SmartDashboard.putNumber("Mode",  mode);
        SmartDashboard.putNumber("Shots Fired", shotsFired);
        SmartDashboard.putBoolean("AtSpeed", weinberger.atSpeed());
        System.out.println(weinberger.atSpeed());
        //Added this so you can see what mode you are in
        air.run();
        //priunsh.run();
        switch(mode){
            case 0:
            {
                while(((leftEncoder.getDistanceFeet()+rightEncoder.getDistanceFeet())/2) < CENTER_DRIVE_DISTANCE) {
                    weinberger.setFireSpeedAuto(0);
                    drive.setMotorSpeed(AUTO_DRIVE_SPEED, AUTO_DRIVE_SPEED);
                }
                if (autoMode == LEFT_SIDE){
                    if (Math.abs(gyro.angle()) <= AUTO_TURN){
                        weinberger.setFireSpeedAuto(0);
                        drive.setMotorSpeed(AUTO_TURN_SPEED, -AUTO_TURN_SPEED);
                    }
                    else
                    { 
                        drive.setMotorSpeed(0.0, 0.0);
                mode++;
                    }
                } else if (autoMode == RIGHT_SIDE){
                    if (Math.abs(gyro.angle()) <= AUTO_TURN){
                        weinberger.setFireSpeedAuto(0);
                        drive.setMotorSpeed(-AUTO_TURN_SPEED, AUTO_TURN_SPEED);
                    }
                    else
                    { 
                        drive.setMotorSpeed(0.0, 0.0);
                        mode++;
                    }
                } else {
                    //drive.setMotorSpeed(0, 0);
                    mode++;
                }
                break;
            }
            case 1:
            {
                weinberger.setFireSpeedAuto(AUTO_SHOT_RPM);
                if (weinberger.atSpeed() || shotsFired >= SHOTS_LOADED){
                    mode++;
                }
                firing = false;
                //Why do you have this firing BOOL?
                break;
            }
            case 2:
            {
                if (shotsFired < SHOTS_LOADED){
                    weinberger.fire();
                    firing = true;
                    SmartDashboard.putBoolean("Firing", firing);
                    //Timer.delay(SHOT_DELAY);
                    //Could the delay be messing things up?
                    mode--;
                    shotsFired++;
                    break;
                } else {
                    //mode = 6;
                    mode++;
                    gyro.reset();
                    break;
                }
            }
            case 3:
            {
                
                if (autoMode == LEFT_SIDE){
                    if (Math.abs(gyro.angle()) <= AUTO_END_TURN){
                        weinberger.setFireSpeedAuto(0);
                        drive.setMotorSpeed(-AUTO_TURN_SPEED, AUTO_TURN_SPEED);
                    }
                    else
                    { 
                        drive.setMotorSpeed(0.0, 0.0);
                        leftEncoder.reset();
                        rightEncoder.reset();
                        mode = 5;
                        //break;
                    }
                } else if (autoMode == RIGHT_SIDE){
                    if (Math.abs(gyro.angle()) <= AUTO_END_TURN){
                        weinberger.setFireSpeedAuto(0);
                        drive.setMotorSpeed(AUTO_TURN_SPEED, -AUTO_TURN_SPEED);
                    }
                    else
                    { 
                        drive.setMotorSpeed(0.0, 0.0);
                        leftEncoder.reset();
                        rightEncoder.reset();
                        mode = 5;
                        //break;
                    }
                } else if (autoMode == CENTER_V2){
                    mode = 4;
                    leftEncoder.reset();
                    rightEncoder.reset();
                    gyro.reset();
                    //break;
                } else {
                mode++;
            }
                break;
            }

            case 4:
            {
               /* while (Math.abs(((leftEncoder.getDistanceFeet()+rightEncoder.getDistanceFeet())/2)) < DRIVE_BACK_DISTANCE) {
                    drive.setMotorSpeed(-AUTO_DRIVE_SPEED, -AUTO_DRIVE_SPEED);
                }
                if (Math.abs(gyro.angle()) <= AUTO_HALF_TURN){
                    weinberger.setFireSpeedAuto(0);
                    drive.setMotorSpeed(AUTO_TURN_SPEED, -AUTO_TURN_SPEED);
           
                }
                else
                { 
                    drive.setMotorSpeed(0.0, 0.0);
                    mode = 6;
                    //break;
                }
                break;*/
                mode = 6;
                break; //put in to stop robot from backing out past the center line
            }
            case 5:
            {
                weinberger.setFireSpeedAuto(0);
                while ((leftEncoder.getDistanceFeet() + rightEncoder.getDistanceFeet())/2 < SIDE_DRIVE_BACK_DISTANCE){
                    drive.setMotorSpeed(AUTO_DRIVE_SPEED, AUTO_DRIVE_SPEED);
                }
                mode++;
                break;
            }
            case 6:
            {
                drive.setMotorSpeed(0, 0);
                weinberger.setFireSpeedAuto(0);
                break;   
            }
                
            
        }
        //Also, there is some encoder stuff in testPeriodic() once we get to that at competition

//        PEACHTREE AUTONOMOUS CODE
        //Set up
//        SmartDashboard.putNumber("RPM", weinberger.getRPM());
//        SmartDashboard.putNumber("Mode", mode);
//        SmartDashboard.putNumber("Shots Fired", shotsFired);
//        SmartDashboard.putBoolean("AtSpeed", weinberger.atSpeed());
//        //System.out.println(weinberger.atSpeed());
//        air.run();
//        priunsh.run();
//        
//        weinberger.setFireSpeedAuto(autoShotSpeed);
//        
//        SmartDashboard.putNumber("EncDistance", distanceDriven);
//        SmartDashboard.putNumber("RCount", rightEncoder.getCount());
//        SmartDashboard.putNumber("LCount", leftEncoder.getCount());
//        
//        if (autoClock.get() >= 13) {
//            weinberger.fire();
//            Timer.delay(SHOT_DELAY);
//        }
//        // State Machine
//        switch (mode) {
//            case 0:
//            {
//                while(time.get() <= AUTO_DRIVE_TIME){
//                    weinberger.setFireSpeedAuto(0);
//                    drive.setMotorSpeed(AUTO_DRIVE_SPEED, AUTO_DRIVE_SPEED);
//                }
//                mode++;
////                if (Math.abs(gyro.angle()) <= AUTO_TURN){
////                    weinberger.setFireSpeedAuto(0);
////                    drive.setMotorSpeed(-AUTO_TURN_SPEED, AUTO_TURN_SPEED);
////                }
////                else
////                { 
////                    drive.setMotorSpeed(0.0, 0.0);
////                    mode++;
////                }
//                break;
//            }
//            case 1:
//                //weinberger.setFireSpeedAuto(0);
//                autoShotSpeed = 0;
//                //Drive State
//                autoMode = DriverStation.getDouble("AutoMode");
//
//                if (autoMode == 1) {
//                    while (distanceDriven <= LEFT_DRIVE_DISTANCE) {
//                        autoShotSpeed = 0;
//                        drive.setMotorSpeed(AUTO_SPEED, AUTO_SPEED);
////                        gyroDrive.withGyroForward();
////                        drive.setMotorSpeed(gyroDrive.getLeftSpeed(), gyroDrive.getRightSpeed());
//                        distanceDriven = (leftEncoder.getDistanceFeet() + rightEncoder.getDistanceFeet()) / 2.0;
//                    }
//                    gyro.reset();
//                    while (angleTurned >= LEFT_TURN) {
//                        autoShotSpeed = 0;
//                        drive.setMotorSpeed(AUTO_SPEED, -AUTO_SPEED);
//                        angleTurned = gyro.angle();
//                    }
//                    mode++;
//                    break;
//                    //forward then turn right
//                } else if (autoMode == 2) {
//                    
//                    while (distanceDriven <= CENTER_DRIVE_DISTANCE) {
//                        //weinberger.setFireSpeedAuto(0);
//                        autoShotSpeed = 0;
//                        drive.setMotorSpeed(AUTO_SPEED, AUTO_SPEED);
////                        gyroDrive.withGyroForward();
////                        drive.setMotorSpeed(gyroDrive.getLeftSpeed(), gyroDrive.getRightSpeed());
//                        distanceDriven = (leftEncoder.getDistanceFeet() + rightEncoder.getDistanceFeet()) / 2.0;
//                    }
//                    drive.setMotorSpeed(0, 0);
//                    mode++;
//                    //weinberger.jag.set(0.2);
//                    break;
//                    //forward
//                } else if (autoMode == 3) {
//                    while (distanceDriven <= RIGHT_DRIVE_DISTANCE) {
//                        //drive.setMotorSpeed(AUTO_SPEED, AUTO_SPEED);
//                        autoShotSpeed = 0;
//                        gyroDrive.withGyroForward();
//                        drive.setMotorSpeed(gyroDrive.getLeftSpeed(), gyroDrive.getRightSpeed());
//                        distanceDriven = (leftEncoder.getDistanceFeet() + rightEncoder.getDistanceFeet()) / 2.0;
//                    }
//                    gyro.reset();
//                    while (angleTurned <= RIGHT_TURN) {
//                        autoShotSpeed = 0;
//                        drive.setMotorSpeed(-AUTO_SPEED, AUTO_SPEED);
//                        angleTurned = gyro.angle();
//                    }
//                    mode++;
//                    break;
//                    // forward then turn left
//                } else {
//                    //no driving
//                    mode++;
//                    break;
//                }
//
//            case 2: //Run the shooter state
//            {
//                //weinberger.setFireSpeedAuto(AUTO_SHOT_RPM);
//                autoShotSpeed = AUTO_SHOT_RPM;
//                if (weinberger.atSpeed()) {
//                    mode++;
//                }
//                firing = false;
//                //Why do you have this firing BOOL?
//                break;
//            }
//            case 3: //shooting state
//            {
//                if (shotsFired < SHOTS_LOADED) {
//                    //weinberger.setFireSpeedAuto(AUTO_SHOT_RPM);
//                    autoShotSpeed = AUTO_SHOT_RPM;
//                    weinberger.fire();
//                    firing = true;
//                    SmartDashboard.putBoolean("Firing", firing);
//                    Timer.delay(SHOT_DELAY);
//                    //Could the delay be messing things up?
//                    mode--;
//                    shotsFired++;
//                    break;
//                } else {
//                    mode++;
//                    break;
//                }
//            }
//            case 4: {
//                gyro.reset();
//                angleTurned = 0.0;
//                mode++;
//                break;
//            }
//            case 5: //end state
//            {
//                //weinberger.setFireSpeedAuto(IDLE_RPM);
//                autoShotSpeed = IDLE_RPM;
////                while (angleTurned <= 175) {
////                    drive.setMotorSpeed(AUTO_SPEED, -AUTO_SPEED);
////                    angleTurned = gyro.angle();
////                }
//                mode++;
//                break;
//
//            }
//            case 6: {
//                //weinberger.setFireSpeedAuto(IDLE_RPM);
//                autoShotSpeed = IDLE_RPM;
//                drive.setMotorSpeed(0, 0);
//                break;
//            }

//        }

    }

    /**
     * This function is called at the beginning of operator control
     */
    public void teleopInit() {
        leftEncoder.stop();
        rightEncoder.stop();
        air.startCompressor();
        priunsh.percentBrightness = 0;
        weinberger.errorMargin = TELEOP_ERROR_LIMIT;
        weinberger.filterSpeed = 0.0;
        //weinberger.setPID();
        weinberger.powerSensor();
//        air.winchIn = false;
//        air.shiftMode(false);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {

        //Set up
        //weinberger.setPID();
        table = NetworkTable.getTable("TitanTable");
        air.run();
        priunsh.run();
        //gyro.run();
        weinberger.incrementDelay = DriverStation.getDouble("IDelay");
        weinberger.increment = DriverStation.getDouble("Increments");
        //weinberger.setFireSpeedAuto(DriverStation.getDouble("RPM"));
        SmartDashboard.putNumber("RPM", weinberger.getRPM());
        weinberger.SHOT_DELAY = DriverStation.getDouble("ShotDelay");

        if (DriverStation.auxStick.getRawButton(HWR.SHOOT) && !last) {
            last = true;
            weinberger.fire();
        } else if (DriverStation.auxStick.getRawButton(HWR.SHOOT) && last) {
            //Do Nothing
        } else {
            last = false;
        }
        //you can add in more buttons for different speed sets
        if (DriverStation.auxStick.getRawButton(HWR.SET_SHOOTER_SPEED)) {
            weinberger.setFireSpeedAuto(DriverStation.getDouble("TeleSpeed"));
        } else if (DriverStation.auxStick.getRawButton(HWR.AUTO_SHOOT)) {
            weinberger.autoShoot(DriverStation.getDouble("TeleSpeed"));
        }  else if (DriverStation.auxStick.getRawButton(5)) {
            weinberger.autoShoot(2800);             
        }
        else {
            weinberger.setFireSpeedAuto(IDLE_RPM);
        }
        
        if(DriverStation.antiBounce(HWR.AUX_JOYSTICK, HWR.EXPEL_BUTTON)) { //antibounce needs to be tested
            weinberger.undoMichael();
        }
        
        if(DriverStation.antiBounce(HWR.AUX_JOYSTICK, 11)) {
            weinberger.unjam();
        }

        SmartDashboard.putNumber("PWM", weinberger.getOutput());
        //NetworkTable Stuff
        
        //THIS IS FOR VISION - put in later
//        try {
//                                targetRightHeight = table.getNumber("topRightY")-table.getNumber("bottomRightY");
//                                targetLeftHeight = table.getNumber("topLeftY")-table.getNumber("bottomLeftY");
//                                targetTopWidth = table.getNumber("topRightX")-table.getNumber("topLeftX");
//                                targetBottomWidth = table.getNumber("bottomRightX")-table.getNumber("bottomLeftX");
//                                }
//                                catch (TableKeyNotDefinedException ex) {
//                                System.out.println("Exception caught");
//                                }
//
//                        targetDistFeet = track.run(targetTopWidth,targetBottomWidth,targetRightHeight,targetLeftHeight);
//                        targetAngleDeg = track.getTargetAngle();
////                        SmartDashboard.putNumber("Target Height in Pixels", targetHeight);
////                        SmartDashboard.putNumber("Target Width in Pixels", targetWidth);
//                        SmartDashboard.putNumber("Target Distance in Feet", targetDistFeet);
//                        SmartDashboard.putNumber("Target Angle in Degrees", targetAngleDeg);
//                        table.putNumber("DistanceInFeet", targetDistFeet);
        //THIS IS FOR VISION TESTING END

        //Drive 
//        if (air.winchIn) {
//            drive.climbMode();
//        } else {
//            drive.driveMode();
//        }
        drive.driveMode();


    }

    /**
     * This function is called at the beginning of test mode
     */
    public void testInit() {
        air.startCompressor();
        leftEncoder.start();
        rightEncoder.start();
        gyro.reset();
        //distanceTracker.resetDistance();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {

        weinberger.setFireSpeedAuto(0.0);
        gyro.run();


//        
//        distanceTracker.updateDistance();
//        if (DriverStation.leftStick.getRawButton(2)) {
//            distanceTracker.resetDistance(); 
//        }
//        SmartDashboard.putNumber("RCount", rightEncoder.getCount());
//        SmartDashboard.putNumber("LCount", leftEncoder.getCount());
//        encoderDist = DriverStation.getDouble("ETD");
//        
//        SmartDashboard.putNumber("Left Distance", leftEncoder.getDistanceMeters());
//        SmartDashboard.putNumber("Right Distance", rightEncoder.getDistanceMeters());
//        if (DriverStation.leftStick.getTrigger()) {
//        if (leftEncoder.getDistanceMeters() < encoderDist || rightEncoder.getDistanceMeters() < encoderDist) {
//            drive.setMotorSpeed(0.2, 0.2);    
//            }
//        else if (leftEncoder.getDistanceMeters() >= encoderDist || rightEncoder.getDistanceMeters() >= encoderDist) {
//            drive.setMotorSpeed(0, 0);
//        }
//        else {
//            System.out.println("Encoder distance error")
//;        }
//        }
//        
//        if (DriverStation.leftStick.getRawButton(4)) {
//            leftEncoder.reset();
//            rightEncoder.reset();
//        }
//        air.run();
        //air.calibrateServos();

    }
}
