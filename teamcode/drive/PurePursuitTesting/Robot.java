package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.os.SystemClock;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MyPosition.worldYPosition;

public class Robot{

    public DcMotor mfl, mfr, mbl, mbr;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        mfl = hardwareMap.dcMotor.get("mfl");
        mfr = hardwareMap.dcMotor.get("mfr");
        mbl = hardwareMap.dcMotor.get("mbl");
        mbr = hardwareMap.dcMotor.get("mbr");

        mfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public double getXPos(){
        return worldXPosition;
    }
    public double getYPos(){
        return worldYPosition;
    }
    public double getAngle_rad(){
        return worldAngle_rad;
    }
    public double getAngle_deg(){
        return Math.toDegrees(worldAngle_rad);
    }

    public static int programStage;

   /* public void updateOdometry(){
        MyPosition.giveMePositions(mfr.getCurrentPosition(), mfl.getCurrentPosition(), mbl.getCurrentPosition());
        Speedometer.update();
    }

    public void giveSpeedAndPositionTelemetry(){
        telemetry.addData("WorldXPosition: ", getXPos());
        telemetry.addData("WorldYPosition: ", getYPos());
        telemetry.addData("WorldRPosition (degrees): ", getAngle_deg());
        telemetry.addData("Speed X: ", Speedometer.getSpeedX());
        telemetry.addData("Speed Y: ", Speedometer.getSpeedY());
        telemetry.addData("Speed R: (degrees / sec)", Speedometer.getDegPerSecond());
    }*/

    public double start_mfl_reading = 0;
    public double start_mfr_reading = 0;
    public double start_mbr_reading = 0;

    public boolean isTeleopDriveAuto = false;

    public static double movement_x;
    public static double movement_y;
    public static double movement_turn;

    public boolean teleopAutoDriveOverride = false;

    public ElapsedTime autoDriveCooldown = new ElapsedTime();

    public boolean autoDriveCooldownWatch = false;

    public double lastUpdateTime = SystemClock.uptimeMillis();

    public ArrayList<CurvePoint> testTeleopPath = new ArrayList<>();

    public CurvePoint testTeleopPathWaypoint1 = new CurvePoint(60,60,0.6,0.6,10,2,2);

    public CurvePoint testTeleopPathWaypoint2 = new CurvePoint(50,110,1,1,10,2,2);

    public CurvePoint startOfPath = new CurvePoint(getXPos(), getYPos(), 0,0,0,0,0);

    public double fl, fr, bl, br;

    public static long currTimeMillis;

    public double robot_width = 20.32;

    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;



        double tl_power_raw = movement_y+movement_turn-movement_x*1.5;
        double bl_power_raw = -movement_y+movement_turn+ movement_x*1.5;
        double br_power_raw = movement_y+movement_turn+movement_x*1.5;
        double tr_power_raw = -movement_y+movement_turn-movement_x*1.5;




        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;


        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        mfl.setPower(tl_power_raw);
        mbl.setPower(bl_power_raw);
        mbr.setPower(br_power_raw);
        mfr.setPower(tr_power_raw);
    }
}