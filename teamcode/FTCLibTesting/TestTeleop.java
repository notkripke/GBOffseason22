package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="wow test", group="Linear Opmode")
public class TestTeleop extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();
        drive.imu.init();
        drive.setPosition(72, 72, 0);

        waitForStart();

        while (!isStopRequested()) { // THIS IS AN OPMODE FOR FIELD-CENTRIC DRIVE. IF ROBOT
            //                          CENTRIC IS DESIRED, CALL DRIVE.APPLYMOVEMENT() IN
            //                          PLACE OF DRIVE.FIELDCENTRICDRIVE()


        setDriveToController();
        drive.fieldCentricDrive();
        drive.updateOdo(20);
        drive.ApplyMovement();
        telemetry.addData("worldPositionX: ", drive.worldXPosition);
        telemetry.addData("worldPositionY: ", drive.worldYPosition);
        telemetry.addData("worldPositionR: ", Math.toDegrees(drive.worldAngle_rad));
        telemetry.update();
        }
    }
}