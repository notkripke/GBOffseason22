package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.RobotMovement2;
import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition;

import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.worldYPosition;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.RobotMovement2.initCurve;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.RobotMovement2.initForMove;

/**
 * Auto is used by autonomous opmodes
 */
public class Auto extends OpMode {


    public boolean stageFinished = true;
    public long stateStartTime = 0;

    public long programStartTime = 0;//time the program starts


    public static int programStage = 0;



    /** BLOCK STARTING VARIABLES */
    public double blockStartingX = 0;
    public double blockStartingY = 0;
    //This is in radians
    public double blockStartingAngle_rad = 0;
    ///////////////////////////////

    /** Debugging Stuff */
    public ArrayList<Double> markedXLocations = new ArrayList<Double>();
    public ArrayList<Double> markedYLocations = new ArrayList<Double>();
    public ArrayList<Double> markedAngleLocations = new ArrayList<Double>();
    public ArrayList<Integer> markedStateIndexes = new ArrayList<Integer>();


    //this will make the robot stop after every move, until the user presses the a button
    public static boolean debugging = false;

    public void startDebugging(){
        debugging = true;
    }


    private boolean inDebugState = false;

    boolean gamepad1_x_last = false;


    //holds the stage we are going to next
    int nextStage = 0;
    public void nextStage(int ordinal) {
        nextStage = ordinal;
        //waits for a if on debug mode
        if(!debugging){
            incrementStage();
            inDebugState = false;
        }

        //go into debug mode
        if(debugging){
            inDebugState = true;
        }
    }

    /** Increments the programStage */
    public void nextStage() {
        nextStage(programStage + 1);

    }
    private void incrementStage() {
        programStage  = nextStage;
        stageFinished = true;
    }

    private void savePoint() {
        //markedXLocations.add(getXPos());
        //markedYLocations.add(getYPos());
        //markedAngleLocations.add(getAngle_deg());
        markedStateIndexes.add(programStage);
    }

    /** called during the init of any stage */
    public void initializeStateVariables() {
        stageFinished = false;
        blockStartingX = worldXPosition;
        blockStartingY = worldYPosition;
        blockStartingAngle_rad = worldAngle_rad;
        stateStartTime = SystemClock.uptimeMillis();
        initForMove();
        initCurve();
    }


    private double startingPos_x = 0;
    private double startingPos_y = 0;
    private double startingPos_angle_rad = 0;

    public void setStartingPosition(double x, double y, double angle_rad){
        startingPos_x = x;
        startingPos_y = y;
        startingPos_angle_rad = angle_rad;
    }

    @Override
    public void init() {
        
    }

    /**
     * Set's our position to the starting postition although this is dumb
     * because who even uses this anyway, we'll reset when we get down
     */

    public void start(){
        //Now we can set our position
        MyPosition.setPosition(startingPos_x,startingPos_y,startingPos_angle_rad);

        stageFinished = true;//need to call initialize state variables
        programStage = 0;//start on the first state
        programStartTime = SystemClock.uptimeMillis();//record the start time of the program
    }

    @Override
    public void loop() {

    }

    public void MainStateMachine() {

    }


    /**
     * This returns true if we have completed a time out
     * @param milliseconds -> time
     * @return
     */
    //public boolean isTimedOut(int milliseconds){
        //return (currTimeMillis - stateStartTime > milliseconds && !debugging);
    //}





    //scale down all motor powers with this
    public static double masterMotorScale = 1.0;

    //the sample we are shifting around
    private int currModifyingSample = 0;

    /**
     * Allows the user to move the auto samples
     */

    }
