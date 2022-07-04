package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.*;


@TeleOp(group = "drive")
@Config
public class  TestTeleop extends LinearOpMode { // 192.168.43.1:8080/dash
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        waitForStart();

        while(!isStopRequested()){



            MyPosition.giveMePositions(robot.mfr.getCurrentPosition(), robot.mfl.getCurrentPosition(),robot.mbr.getCurrentPosition() );
        }


    }
}


