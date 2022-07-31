package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public void updateOdo(){
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
    }


    // NOW ENTERING THE PURE PURSUIT ZONE ---------------------------------------

}
