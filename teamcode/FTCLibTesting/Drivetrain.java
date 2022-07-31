package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;

public class Drivetrain {

    public static double movement_x;
    public static double movement_y;
    public static double movement_turn;
    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;


    Telemetry tele;

    public DcMotor mfl, mfr, mbl, mbr;

    public Drivetrain()
    {
        tele = telemetry;

        mfl = hardwareMap.dcMotor.get("mfl");
        mfr = hardwareMap.dcMotor.get("mfr");
        mbl = hardwareMap.dcMotor.get("mbl");
        mbr = hardwareMap.dcMotor.get("mbr");

        mfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    RevIMU imu = new RevIMU(hardwareMap);
    /**
     * This will accept a user-given position in inches and degrees
     * @param x in degrees
     * @param y in degrees
     * @param heading in degrees
     * @return a new Pose2d in meters and radians, to be used by the odometry
     */

    public Pose2d ItoMPose2d(double x, double y, double heading){
        double adj_x = x * 39.37;
        double adj_y = y * 39.37;
        double adj_heading = Math.toRadians(heading);

        return new Pose2d(adj_x, adj_y,adj_heading);
    }

    Vector2d mfl_location = new Vector2d(0.1778, 0.1778);
    Vector2d mfr_location = new Vector2d(0.1778, -0.1778);
    Vector2d mbl_location = new Vector2d(-0.1778, 0.1778);
    Vector2d mbr_location = new Vector2d(-0.1778, -0.1778);

    /**
     * @param x Robot X position in inches
     * @param y Robot Y position in inches
     * @param heading Robot heading in degrees
     */
    public void setPosition(double x, double y, double heading){
        worldXPosition = x;
        worldYPosition = y;
        worldAngle_rad = Math.toRadians(heading);
        gyro_loop_index = 0;
    }

    double old_time = 0;
    double new_time = 0;
    double oldMFL = 0;
    double oldMFR = 0;
    double oldMBL = 0;
    double oldMBR = 0;
    double motorCPR = 384.5;
    double wheel_radius = 4.0;
    double lx = 9.50;
    double ly = 7.50;

    public static double scalePrediction = 1.0; //TUNE ME
    public static double ySlipDistanceFor1InPS = 0.14 * scalePrediction;
    public static double xSlipDistanceFor1InPS = 0.14 * scalePrediction;
    //radians the robot slips when going 1 radian per second
    public static double turnSlipAmountFor1RPS = 0.1 * scalePrediction;
    public static double relativeY;
    public static double relativeX;
    public static double worldAngleLast;
    public static int gyro_loop_index = 0;
    /**
     *This will update the odometry positions using IMU and encoder data
     * @return no direct return, but will update worldX, worldY positions and worldAngle_rad
     */
    public void updateOdo(int gyro_loop_period){
        new_time = SystemClock.uptimeMillis();

        double time = new_time - old_time;

        worldAngleLast = worldAngle_rad;

        //GET WHEEL VELOCITIES IN RAD / SEC
        double newMFL = mfl.getCurrentPosition();
        double deltaMFL = newMFL - oldMFL;
        double mflV = (deltaMFL / time)/(motorCPR / (2 * Math.PI));
        double newMFR = mfr.getCurrentPosition();
        double deltaMFR = newMFR - oldMFR;
        double mfrV = (deltaMFR / time)/(motorCPR / (2 * Math.PI));
        double newMBL = mbl.getCurrentPosition();
        double deltaMBL = newMBL - oldMBL;
        double mblV = (deltaMBL / time)/(motorCPR / (2 * Math.PI));
        double newMBR = mbr.getCurrentPosition();
        double deltaMBR = newMBR - oldMBR;
        double mbrV = (deltaMBR / time)/(motorCPR / (2 * Math.PI));
        //---------------------------------------------------------

        double vY = (mflV + mfrV + mblV + mbrV) * (wheel_radius/4);
        double vX = (-mflV + mfrV + mblV - mbrV) * (wheel_radius/4);
        double vR = (-mflV + mfrV - mblV + mbrV) * (wheel_radius / (4*(lx+ly)));

        double deltaX = (vX * time) - (xSlipDistanceFor1InPS * vX);
        double deltaY = (vX * time) - (ySlipDistanceFor1InPS * vY);
        double deltaR = (vX * time) - (turnSlipAmountFor1RPS * vR);

        double resultant = Math.atan2(deltaY, deltaX);

        double angleIncrement = (deltaMFL-deltaMFR) / (motorCPR / (2 * Math.PI));

        if(Math.abs(angleIncrement) > 0.25){ // 0.25 is buffer for change. right now this is 1/4 inch
            //gets the radius of the turn we are in
            double radiusOfMovement = ((deltaMFL / (motorCPR / (2 * Math.PI)))
                                        +(deltaMFR / (motorCPR / (2 * Math.PI)))/(2*angleIncrement));
            //get the radius of our straifing circle
            double radiusOfStraif = deltaX/angleIncrement;

            relativeY = (radiusOfMovement * Math.sin(resultant)) - (radiusOfStraif * (1 - Math.cos(resultant)));

            relativeX = radiusOfMovement * (1 - Math.cos(resultant)) + (radiusOfStraif * Math.sin(resultant));

        }
        worldXPosition += (Math.cos(worldAngleLast) * relativeY) + (Math.sin(worldAngleLast) *
                relativeX);
        worldYPosition += (Math.sin(worldAngleLast) * relativeY) - (Math.cos(worldAngleLast) *
                relativeX);
        worldAngle_rad += deltaR;


        gyro_loop_index += 1;

        if(gyro_loop_index == 0){
            imu.relativeHeading = worldAngle_rad;
        }

        if(gyro_loop_index == gyro_loop_period){
            worldAngle_rad = imu.getHeading();
            gyro_loop_index = 1;
        }
    }
    long lastUpdateTime;

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

    public void fieldCentricDrive(){
// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -movement_y,
                -movement_x
        ).rotated(-Math.toDegrees(worldAngle_rad));
        movement_x = input.getX();
        movement_y = input.getY();
        ApplyMovement();
    }


    // NOW ENTERING THE PURE PURSUIT ZONE ---------------------------------------

}
