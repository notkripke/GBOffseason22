package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.IMU;

public class MyPosition {

    public static Robot myRobot;
    public static double moveScalingFactor = 12.56064392;
    public static double turnScalingFactor = 35.694;
    public static double auxScalingFactor = 12.48;
    public static double auxPredictionScalingFactor = 0.92;


    public static double wheelLeftLast = 0.0;
    public static double wheelRightLast = 0.0;
    public static double wheelAuxLast = 0.0;

    public static double lastAngle = 0.0;

    public static double worldXPosition = 0.0;

    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;

    public static double worldXPositionOld = 0.0;
    public static double worldYPositionOld = 0.0;


    public static double currPos_l = 0;
    public static double currPos_r = 0;
    public static double currPos_a = 0;
    public static double currPos_rg = 0;



    //stuff for reading the angle in an absolute manner
    public static double wheelLeftInitialReading = 0.0;
    public static double wheelRightInitialReading = 0.0;
    public static double lastResetAngle = 0.0;//this is set when you reset the position


    //use this to get how far we have traveled in the y dimension this update
    public static double currentTravelYDistance = 0.0;


    public static void initialize(double l, double r,double a, Robot myRobot){
        MyPosition.myRobot = myRobot;
        currPos_l = l;
        currPos_r = r;
        currPos_a = a;
        update();
    }

    public static void giveMePositions(double l, double r, double a){
        currPos_l = l;
        currPos_r = r;
        currPos_a = a;
        update();
    }

    private static void update(){
        PositioningCalculations();
    }


    /**
     * Makes sure an angle is in the range of -180 to 180
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }

    /**
     * Updates our position on the field using the change from the encoders
     */
    public static void PositioningCalculations(){
        double wheelLeftCurrent = -currPos_l;
        double wheelRightCurrent= currPos_r;
        double wheelAuxCurrent = currPos_a;

        //compute how much the wheel data has changed
        double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


        //get the real distance traveled using the movement scaling factors
        double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
        double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
        double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;

        //get how much our angle has changed
        double angleIncrement = (wheelLeftDelta-wheelRightDelta)*turnScalingFactor/100000.0;
        //myRobot.telemetry.addLine("Angle increment is " +
          //      (angleIncrement > 0 ? "POSITIVE" : "NEGATIVE"));


        //but use absolute for our actual angle
        double wheelRightTotal = currPos_r-wheelRightInitialReading;
        double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);

        double worldAngleLast = worldAngle_rad;
        worldAngle_rad = AngleWrap(((wheelLeftTotal-wheelRightTotal)*turnScalingFactor/100000.0) + lastResetAngle);

        //get the predicted amount the straif will go
        double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
        //now subtract that from the actual
        double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


        //relativeY will by defa
        double relativeY = (wheelLeftDeltaScale + wheelRightDeltaScale)/2.0;
        double relativeX = r_xDistance;



        //if angleIncrement is > 0
        if(Math.abs(angleIncrement) > 0){
            //gets the radius of the turn we are in
            double radiusOfMovement = (wheelRightDeltaScale+wheelLeftDeltaScale)/(2*angleIncrement);
            //get the radius of our straifing circle
            double radiusOfStraif = r_xDistance/angleIncrement;





            relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStraif * (1 - Math.cos(angleIncrement)));

            relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStraif * Math.sin(angleIncrement));

        }



        worldXPosition += (Math.cos(worldAngleLast) * relativeY) + (Math.sin(worldAngleLast) *
                relativeX);
        worldYPosition += (Math.sin(worldAngleLast) * relativeY) - (Math.cos(worldAngleLast) *
                relativeX);


        Speedometer.yDistTraveled += relativeY;
        Speedometer.xDistTraveled += r_xDistance;



        //save the last positions for later
        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;


        //save how far we traveled in the y dimension this update for anyone that needs it
        currentTravelYDistance = relativeY;
    }

    public static void PositioningCalculationsNew(){
        double wheelLeftCurrent = -currPos_l;
        double wheelRightCurrent= currPos_r;
        double wheelAuxCurrent = currPos_a;

        //compute how much the wheel data has changed
        double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


        //get the real distance traveled using the movement scaling factors
        double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
        double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
        double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;

        //get how much our angle has changed

        //myRobot.telemetry.addLine("Angle increment is " +
        //      (angleIncrement > 0 ? "POSITIVE" : "NEGATIVE"));


        //but use absolute for our actual angle
        double wheelRightTotal = currPos_r-wheelRightInitialReading;
        double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);

        double worldAngleLast = worldAngle_rad;
        worldAngle_rad = Math.toRadians(IMU.getAngle());
        double angleIncrement = ((worldAngle_rad - worldAngleLast) / (2*Math.PI))*myRobot.robot_width;

        //get the predicted amount the straif will go
        double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
        //now subtract that from the actual
        double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


        //relativeY will by defa
        double relativeY = (wheelLeftDeltaScale + wheelRightDeltaScale)/2.0;
        double relativeX = r_xDistance;



        //if angleIncrement is > 0
        if(Math.abs(angleIncrement) != 0){
            //gets the radius of the turn we are in
            double radiusOfMovement = (wheelRightDeltaScale+wheelLeftDeltaScale)/(2*angleIncrement);
            //get the radius of our straifing circle
            double radiusOfStraif = r_xDistance/angleIncrement;





            relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStraif * (1 - Math.cos(angleIncrement)));

            relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStraif * Math.sin(angleIncrement));

        }



        worldXPosition += (Math.cos(worldAngleLast) * relativeY) + (Math.sin(worldAngleLast) *
                relativeX);
        worldYPosition += (Math.sin(worldAngleLast) * relativeY) - (Math.cos(worldAngleLast) *
                relativeX);


        Speedometer.yDistTraveled += relativeY;
        Speedometer.xDistTraveled += r_xDistance;



        //save the last positions for later
        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;


        //save how far we traveled in the y dimension this update for anyone that needs it
        currentTravelYDistance = relativeY;
    }

    /**
     * Updates our position on the field using the change from the encoders
     */
    public static void PositioningCalculationsOld(){
        double wheelLeftCurrent = currPos_l;
        double wheelRightCurrent= -currPos_r;
        double wheelAuxCurrent = currPos_a;

        //compute how much the wheel data has changed
        double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


        //get the real distance traveled using the movement scaling factors
        double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
        double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
        double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;



        //get how much our angle has changed
        double angleIncrement = (wheelLeftDelta-wheelRightDelta)*turnScalingFactor/100000.0;



        //but use absolute for our actual angle
        double wheelRightTotal = currPos_r-wheelRightInitialReading;
        double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);
        worldAngle_rad = AngleWrap(((wheelLeftTotal-wheelRightTotal)*turnScalingFactor/100000.0) + lastResetAngle);




        //relative y translation
        double r_yDistance = (wheelRightDeltaScale+wheelLeftDeltaScale)/2;


        double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
        double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


        worldXPosition += (Math.cos(worldAngle_rad) * r_yDistance) + (Math.sin(worldAngle_rad) *
                r_xDistance);
        worldYPosition += (Math.sin(worldAngle_rad) * r_yDistance) - (Math.cos(worldAngle_rad) *
                r_xDistance);



        Speedometer.yDistTraveled += r_yDistance;
        Speedometer.xDistTraveled += r_xDistance;



        //save the last positions for later
        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;


        //save how far we traveled in the y dimension this update for anyone that needs it
        //currently the absolute control of the collector radius uses it to compensate for
        //robot movement
        currentTravelYDistance = r_yDistance;

    }
    public static double subtractAngles(double angle1, double angle2){
        return AngleWrap(angle1-angle2);
    }




    /**USE THIS TO SET OUR POSITION**/
    public static void setPosition(double x,double y,double angle){
        worldXPosition = x;
        worldYPosition = y;
        worldAngle_rad= angle;

        worldXPositionOld = x;
        worldYPositionOld = y;

        //remember where we were at the time of the reset
        wheelLeftInitialReading = currPos_l;
        wheelRightInitialReading = currPos_r;
        lastResetAngle = angle;
    }

    ////////////////////////////////////////////////////////////////////////////////


    public static float AngleWrap(float angle){
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }
}