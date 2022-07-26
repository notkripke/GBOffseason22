package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Speedometer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.myRobot;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot.currTimeMillis;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot.movement_x;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot.movement_y;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot.movement_turn;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot.programStage;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.AngleWrap;

/**
 * Measure Slip:
 * Takes measurments for how far the robot slips given different speeds.
 */
@Disabled
@Autonomous(name = "SlipCalibrator", group = "auto")
public class SlipCalibrator extends Auto {

    Gamepad gamepad1 = new Gamepad();


    //speed the robot will go before the slide
    private static double GOING_FORWARDS_SPEED = 0.3;
    private static double GOING_SIDEWAYS_SPEED = 0.3;
    private static double TURNING_SPEED = 0.4;


    //how long the robot will accelerate for before performing the slip
    private static final int ACCELERATION_TIME = 1000;
    //how long the robot will wait until taking the measurement
    private static final int SLIP_TIME = 1000;



    //how fast (meters per second) we were going at end of movement_y
    private double movement_speed_y = 0.0;
    private double movement_speed_x = 0.0;
    private double turn_speed = 0.0;

    public enum progStates{

        allowingUserControl1,
        goingForwards,//goes fast forwards
        measuringForwardsSlip,//stops, measuring the slip distance
        allowingUserControl2,//allows user to control robot to get ready
        goingSideways,//goes fast sideways
        measuringSidewaysSlip,//stops, measuring the slip distance
        allowingUserControl3,//allows user to control robot to get ready
        turning,//turns fast
        measuringTurningSlip,//stops, measures radians turned
        endProgram//does nothing, so values are still displayed
    }

    public void init(){

        //start
        programStage = progStates.allowingUserControl1.ordinal();

        //since we are measuring, start at 0,0,0
        MyPosition.setPosition(0,0,Math.toRadians(0));
    }

    public void init_loop(){
        currTimeMillis = SystemClock.uptimeMillis();
    }

    @Override
    public void start(){
        super.start();
    }

    public void loop(){
        telemetry.addLine("Y power: " + GOING_FORWARDS_SPEED);
        telemetry.addLine("X power: " + GOING_SIDEWAYS_SPEED);
        telemetry.addLine("Turn power: " + TURNING_SPEED);

        currTimeMillis = SystemClock.uptimeMillis();


        if(gamepad1.dpad_up){
            GOING_FORWARDS_SPEED += 0.02;
        }
        if(gamepad1.dpad_down){
            GOING_FORWARDS_SPEED -= 0.02;
        }

        if(gamepad1.dpad_right){
            GOING_SIDEWAYS_SPEED += 0.02;
        }
        if(gamepad1.dpad_left){
            GOING_SIDEWAYS_SPEED -= 0.02;
        }
        if(gamepad1.y){
            TURNING_SPEED += 0.02;
        }
        if(gamepad1.a){
            TURNING_SPEED -= 0.02;
        }


        telemetry.addLine("y speed: " + movement_speed_y);
        telemetry.addLine("x speed: " + movement_speed_x);
        telemetry.addLine("turn speed: " + turn_speed);
    }

    public void MainStateMachine() {
        //display the values that were calibrated
        telemetry.addData("ySlipDistanceFor1CMPS",Speedometer.ySlipDistanceFor1CMPS);
        telemetry.addData("xSlipDistanceFor1CMPS",Speedometer.xSlipDistanceFor1CMPS);
        telemetry.addData("turnSlipAmountFor1RPS",Speedometer.turnSlipAmountFor1RPS);

        if(programStage == progStates.allowingUserControl1.ordinal() ||
                programStage == progStates.allowingUserControl2.ordinal() ||
                programStage == progStates.allowingUserControl3.ordinal()){
            myRobot.movement_turn = gamepad1.right_stick_x;
            myRobot.movement_y = gamepad1.left_stick_y;
            myRobot.movement_x = gamepad1.left_stick_x;
            myRobot.ApplyMovement();
            telemetry.addLine("PRESS A TO CONTINUE");
            if(gamepad1.a){

                programStage += 1;
            }
        }



//        if(programStage == progStates.extendingCollectorExtension.ordinal()){
//            if(stageFinished){
//                initializeStateVariables();
//            }
//            myCollector.setExtensionPowerRaw(COLLECTOR_POWER);
//            if(myCollector.getExtensionPercent() > 0.6){
//                nextStage();
//            }
//        }
//
//        if(programStage == progStates.measuringCollectorExtensionSlip.ordinal()){
//            if(stageFinished){
//                collector_speed = myCollector.getExtensionCurrSpeed();
//                collector_length_before_slip = myCollector.getCurrExtensionDistFromCenter();
//                collector_percent_before_slip = myCollector.getTiltPercent();
//                initializeStateVariables();
//            }
//
//
//            //this will apply active braking to stop the collector if it's still going forwards
//            if(myCollector.getExtensionCurrSpeed() > 0){
//                myCollector.setExtensionPowerRaw(-1.0);
//            }else{
//                myCollector.setExtensionPowerRaw(0.0);
//            }
//
//            if(currTimeMillis-stateStartTime > SLIP_TIME){
//                myCollector.calibrateSlipDistance(myCollector.getCurrExtensionDistFromCenter() - collector_length_before_slip,collector_speed);
//                nextStage();
//            }
//
//        }


        if(programStage == progStates.goingForwards.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movement_y = GOING_FORWARDS_SPEED;
            movement_x = 0.0;
            movement_turn = 0.0;
            if(currTimeMillis - stateStartTime > ACCELERATION_TIME){
                movement_speed_y = Speedometer.getSpeedY();
                myRobot.movement_x = 0;
                myRobot.movement_turn = 0;
                programStage += 1;
            }
        }
        if(programStage == progStates.measuringForwardsSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }


            double distance = 0; //Math.sqrt(Math.pow(/*getXPos()-blockStartingX,2) + Math.pow(getYPos()-blockStartingY,2*/, 0));


            if(currTimeMillis-stateStartTime > SLIP_TIME){
                Speedometer.ySlipDistanceFor1CMPS = distance/movement_speed_y;
                myRobot.movement_turn = 0;
                myRobot.movement_x = 0;
                myRobot.movement_y = 0;
                nextStage();
            }
        }

        if(programStage == progStates.goingSideways.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movement_y = 0.0;
            movement_x = GOING_SIDEWAYS_SPEED;
            movement_turn = 0.0;

            if(currTimeMillis - stateStartTime > ACCELERATION_TIME){
                movement_speed_x = Speedometer.getSpeedX();
                myRobot.movement_x = 0;
                nextStage();
            }
        }
        if(programStage == progStates.measuringSidewaysSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            double distance = 0; //Math.sqrt(Math.pow(getXPos()-blockStartingX,2) + Math.pow(getYPos()-blockStartingY,2));

            if(currTimeMillis-stateStartTime > SLIP_TIME){
                Speedometer.xSlipDistanceFor1CMPS = distance/movement_speed_x;
                nextStage();
            }
        }
        if(programStage == progStates.turning.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movement_y = 0.0;
            movement_x = 0.0;
            movement_turn = TURNING_SPEED;

            if(currTimeMillis - stateStartTime > ACCELERATION_TIME){
                turn_speed = Speedometer.getRadPerSecond();
                myRobot.movement_x = 0;
                myRobot.movement_y = 0;
                nextStage();
            }
        }
        if(programStage == progStates.measuringTurningSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            double radsTurned =0; //AngleWrap(getAngle_rad()-blockStartingAngle_rad);
            telemetry.addData("degreesTurned: ",Math.toDegrees(radsTurned));
            telemetry.addData("Turn speed deg:" ,Math.toDegrees(turn_speed));
            if(currTimeMillis-stateStartTime > SLIP_TIME){
                Speedometer.turnSlipAmountFor1RPS = radsTurned/turn_speed;
                nextStage(0);
            }
        }
    }


}

