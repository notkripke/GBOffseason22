package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import android.os.SystemClock;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.RobotMovement2.pointAngle;

public class Robot {

    public DcMotor mfl, mfr, mbl, mbr;

    public Robot robot(HardwareMap hardwareMap, Telemetry telemetry){
        mfl = hardwareMap.dcMotor.get("mfl");
        mfr = hardwareMap.dcMotor.get("mfr");
        mbl = hardwareMap.dcMotor.get("mbl");
        mbr = hardwareMap.dcMotor.get("mbr");

        mfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int retur = 0;
        return this.robot(hardwareMap, telemetry);
    }

    public static double movement_x;
    public static double movement_y;
    public static double movement_turn;

    public double lastUpdateTime = SystemClock.uptimeMillis();

    private double fl, fr, bl, br;


    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;



        double tl_power_raw = movement_y-movement_turn+movement_x*1.5;
        double bl_power_raw = movement_y-movement_turn- movement_x*1.5;
        double br_power_raw = -movement_y-movement_turn-movement_x*1.5;
        double tr_power_raw = -movement_y-movement_turn+movement_x*1.5;




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

    public CurvePoint poop = new CurvePoint(0,0,0,0,0,0,0,0);



}

