package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class GorillabotsCentral extends LinearOpMode {
    public Robot robot;
    public MyPosition pos;

    public void initializeComponents(){
        robot = new Robot(hardwareMap, telemetry);
        pos = new MyPosition();
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
