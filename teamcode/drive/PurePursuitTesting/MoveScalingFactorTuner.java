package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;


@TeleOp(name="TestAutoIterative", group="teleop")
public class MoveScalingFactorTuner extends OpMode

    /* This program is to find the optimal values for the ScalingFactors in MyPosition. The factors
    are multipliers applied to the encoder delta values to compute how much actual distance was theoretically
    traveled. The program has different modes, one for moveScalingFactor, one for turnScalingFactor, and one for
    auxScalingFactor (move and aux will be very similar).

                                                INSTRUCTIONS

            Do one at a time, then restart program. Move robot a predetermined, known amount (like 30 cm, longer the better).
            The reading on telemetry is the raw encoder delta readings. Divide the reading by the distance covered, then presto.
            Incorporate the number into MyPosition.
     */




{

    Robot robot = new Robot();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        MyPosition.setPosition(200,200,Math.toRadians(90)); //starting pos estimate
        robot.mfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.mfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.mbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.mbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    robot.start_mbr_reading = robot.mbr.getCurrentPosition();
    robot.start_mfl_reading = robot.mfl.getCurrentPosition();
    robot.start_mfr_reading = robot.mfr.getCurrentPosition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    telemetry.addData("deltaMovement L+R / 2: ", ((robot.mfl.getCurrentPosition() - robot.start_mfl_reading) +
                                            (robot.mfr.getCurrentPosition() - robot.start_mfr_reading)) / 2);

    telemetry.addData("aux version thingy: ", robot.mbr.getCurrentPosition() - robot.start_mbr_reading);

    telemetry.addData("turn version thingy: ", (robot.mfl.getCurrentPosition() - robot.start_mfl_reading)
                                                            -(robot.mfr.getCurrentPosition() - robot.start_mfr_reading));

    robot.giveSpeedAndPositionTelemetry();
    telemetry.update();

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.movement_x = 0;
        robot.movement_y = 0;
        robot.movement_turn = 0;
        robot.ApplyMovement();
    }

}
