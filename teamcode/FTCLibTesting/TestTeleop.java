package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="wow test", group="Linear Opmode")
public class TestTeleop extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();


        waitForStart();

        while (!isStopRequested()) {


        }
    }
}