package org.firstinspires.ftc.teamcode.Teamcode.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Lesson", group="teleop")
public class Lesson extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor = hardwareMap.dcMotor.get("motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double a = 4.2;

        waitForStart();

        while(!isStopRequested()) {
            motor.setPower(gamepad1.right_trigger);
        }


    }
}
