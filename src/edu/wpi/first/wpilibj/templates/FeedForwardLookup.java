/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author shoyru
 */
public class FeedForwardLookup {
    
    double[] rpmArray = { 1000,1200,1500,2000,2500,3000,3500};
    double[] FFArray = {0.00000034, 0.00000033, 0.00000021, 0.00000018, 0.00000016, 0.0000002, 0.00000023};
    double finalFF;
    int arrayIndex;
    
    public FeedForwardLookup() {
    
}
     /**
     * Linear Interpolating code
     * 
     * @param rpm distance to target
     * @param upRPM higher interpolating distance
     * @param downRPM lower interpolating distance
     * @param upFF higher interpolating speed
     * @param downVelocity lower interpolating speed
     * @return double calcSpeed (speed at which ball should be launched) 
     */
    public double interpolator (double rpm, double upRPM, double downRPM,double upFF,
                        double downFF) {
                
        if ((upRPM-downRPM) == 0) {
            System.out.println("Dividing by zero");
            return 0;
        }
        double calcFF = (rpm - downRPM)/(upRPM - downRPM) * (upFF -
                            downFF) + downFF;
        
        return calcFF;
    }
    
    /**
     * Takes distance and gives a velocity based on a flexible lookup table
     * 
     * Trims the value to account for changes in ball density at competition
     * @param distance distance to target
     * @return velocity (is actually an RPM)
     */
    public double flexInterpolator(double rpm){

        
        if (rpmArray.length != FFArray.length) {
            System.out.println("Look up tables not the same length");
            return 0.0;
        } else if (rpmArray.length < 2) {
            System.out.println("Lookup tables must have more than 2 points");
            return 0.0;
        }
        
        if (rpm >= rpmArray[rpmArray.length-1]) {
                
            finalFF = FFArray[FFArray.length-1];
            System.out.println("Distance > max range; Velocity set to max");

        } else if (rpm <= rpmArray[0]) {

            finalFF = FFArray[0];
            System.out.println("Distance < min range; velocity set to min");

        } else {

            // within the table bounds so find the lower index of the array that brackets
            // or straddles the target value. Need that index and the one directly above it
            // this simple search is inefficient and only good for short tables
            // future: could re-implement the for loop as a binary search

            for (int i=0; i<rpmArray.length-1; i++) {
                if (rpm >= rpmArray[i] && rpm < rpmArray[i+1]) {
                    arrayIndex = i;
                    break; //found it so leave the loop early
                }
        }

        finalFF = interpolator( rpm, rpmArray[arrayIndex], rpmArray[arrayIndex+1],
                                                    FFArray[arrayIndex], FFArray[arrayIndex+1] ); 
        }
        //System.out.println("FF: " + finalFF);
        return finalFF;
    }
    
   
    /**
     * Interpolates a value into a lookup table (LUT)
     * 
     * otherwise basically the same as above but can be used in future years
     * intended to be fairly general purpose
     * the arrays can vary in size and are passed in
     *
     * @param distance   value at which the corresponding output is desired
     * @param inArray   array of LUT values for the input
     * @param outArray   array of LUT values for the output which correspond to the input
     * @return outVelocity   result of table interpolation
     *
     * @author Mentor Pete
     */
    public double tableInterpolator(double distance, double[] inArray, double[] outArray){
        double outVelocity;
        
	// check that arrays are reasonable before using them
	if (inArray.length != outArray.length) {
            System.out.println("ERROR: Lookup table arrays not same length!");
            return 0.0;

        } else if (inArray.length < 2) {
            System.out.println("ERROR: Lookup table must have at least 2 points!");
            return 0.0;
	}

        if (distance >= inArray[inArray.length-1]) {
            outVelocity = outArray[outArray.length-1];
            System.out.println("Input value > max; output set to max");

        } else if (distance <= inArray[0]) {
            outVelocity = outArray[0];
            //System.out.println("Input value < min; output set to min");

        } else {
            // within the table bounds so find the lower index of the array that brackets
            // or straddles the target value. Need that index and the one directly above it.
            // This simple search is inefficient and only good for short tables.
            // future: could re-implement the for loop as a binary search

            for (int i=0; i<inArray.length-1; i++) {
                if (distance >= inArray[i] && distance < inArray[i+1]) {
                    arrayIndex = i;
                    break; //found it so leave the loop early
                }
            }

            outVelocity = interpolator( distance, inArray[arrayIndex], inArray[arrayIndex+1],
                                                  outArray[arrayIndex], outArray[arrayIndex+1] ); 
        }

        return outVelocity;
    }

 /**
    * Test method containing test cases for interpolation
    *
    * There are MUCH better ways to do this than is shown here
    * but this is simple and expedient
    * 
    * For more info, have a look at jUnit for unit testing in java
    *
    * "Any program feature without an automated test simply doesn't exist"
    * from:  Extreme Programming Explained, Kent Beck, pg 57
    *
    * @return none 
    *
    * @author MentorPete, Sarang
    */

//   void testInterpolationRoutines() {
//
//       //@todo: sarang to finish this
//
//       //can test interpolator2 which is used by both flexinterpolator2 & tableInterpolator
//
//       double y1 = interpolator(4.0, 1.0, 11.0, 100.0, 200.0);
//       if ( (y1 - 130.0) < 0.1) {
//           //System.out.println("Interpolator test passed");
//       } else {
//           //System.out.println("Interpolator test failed");
//       }
//        
//       
//       
//       //flex interpolator test
//       // first zone
//       // test the following with
//       // disarray = 0,4,15,18,21,30
//       // velarray = 0,16,225,324,441,900
//       double test = flexInterpolator(3.0);
//       if (velArray[1] - test*test <= 0.5) {
//           //System.out.println("FlexInterp test 1 success:" + test);
//       } else {
//           //System.out.println("FlexInterp test 1 failed");
//       }
//       // second zone
//       test = flexInterpolator(7.0);
//       if (velArray[2] - test*test <= 0.5) {
//           //System.out.println("FlexInterp test 2 success:" + test);
//       } else {
//           //System.out.println("FlexInterp test 2 failed");
//       }
//       // third zone
//       test = flexInterpolator(16);
//       if (velArray[3] - test*test <= 0.5) {
//           //System.out.println("FlexInterp test 3 success:" + test);
//       } else {
//           //System.out.println("FlexInterp test 3 failed");
//       }
//       // fourth zone
//       test = flexInterpolator(19);
//       if (velArray[4] - test*test <= 0.5) {
//           //System.out.println("FlexInterp test 4 success:" + test);
//       } else {
//           //System.out.println("FlexInterp test 4 failed");
//       }
//       // fifth zone
//       test = flexInterpolator(25.0);
//       if (velArray[5] - test*test <= 0.5) {
//           //System.out.println("FlexInterp test 5 success:" + test);
//       } else {
//           //System.out.println("FlexInterp test 5 failed");
//       }
//       // less than table
//       test = flexInterpolator(-5.0);
//       if (test == velArray[0]) {
//           //System.out.println("FlexInterp less than success:" + test);
//       } else {
//           //System.out.println("FlexInterp less than failed");
//       }
//       // greater than table
//       test = flexInterpolator(40.0);
//       if (test == velArray[velArray.length-1]) {
//           //System.out.println("FlexInterp test greater than success:" + test);
//       } else {
//           //System.out.println("FlexInterp test greater than failed");
//       }
//       //exactly on a point
//       test = flexInterpolator(15.0);
//       if (velArray[3] - test*test <= 0.5) {
//           //System.out.println("FlexInterp test exactly success:" + test);
//       } else {
//           //System.out.println("FlexInterp test 2exactly failed");
//       }
//       
//       
//       // test the tableInterpolator
//       // many more test cases are possible, but these should give some confidence
//
//       double[] xArray = {  1.0,    3.0,   5.0,   7.0,   9.0};
//       double[] yArray = {101.0, 109.00, 125.0, 149.0, 181.0}; //y = 100.0 + x**2
//       double xIn;
//       double yOut;
//
//       // TEST - pt in table between points
//       xIn = 4.0;
//       yOut = tableInterpolator(xIn, xArray, yArray);
//       
//       if ( (yOut-(100.0+xIn*2)) < 1.5 ) {
//           //System.out.println("tableInterpolator test passed");
//       } else {
//           //System.out.println("tableInterpolator test failed");
//       }
//
//       // TEST - pt in table exactly at a table point
//       xIn = 5.0;
//       yOut = tableInterpolator(xIn, xArray, yArray);
//       if ( (yOut - (100.0+ xIn*2)) < 1.5 ) {
//           //System.out.println("tableInterpolator test passed");
//       } else {
//           //System.out.println("tableInterpolator test failed");
//       }
//
//       // TEST - pt in last range of table
//       xIn = 8.0;
//       yOut = tableInterpolator(xIn, xArray, yArray);
//       if ( (yOut-(100.0+xIn*2)) < 1.5 ) {
//           //System.out.println("tableInterpolator test passed");
//       } else {
//           //System.out.println("tableInterpolator test failed");
//       }
//
//       // TEST - pt below input range
//       xIn = 0.5;
//       yOut = tableInterpolator(xIn, xArray, yArray);
//       if ( yOut == yArray[0] ) {
//           //System.out.println("tableInterpolator test passed");
//       } else {
//           //System.out.println("tableInterpolator test failed");
//       }
//
//       //TEST - pt above input range
//       xIn = 10.0;
//       yOut = tableInterpolator(xIn, xArray, yArray);
//       if ( yOut == yArray[yArray.length-1] ) {
//           //System.out.println("tableInterpolator test passed");
//       } else {
//           //System.out.println("tableInterpolator test failed");
//       }
//
//   }
}
