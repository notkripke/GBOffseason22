package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;


@TeleOp(name="TestAutoIterative", group="teleop")
public class TestTeleopIterative extends OpMode
{
    public String program_stage = "start";

    Robot robot = new Robot();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        MyPosition.setPosition(200,200,Math.toRadians(90)); //starting pos estimate

        robot.testTeleopPath.add(robot.testTeleopPathWaypoint1);
        robot.testTeleopPath.add(robot.testTeleopPathWaypoint2);

        robot.teleopAutoDriveOverride = false;

        robot.autoDriveCooldown.reset();
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
    robot.isTeleopDriveAuto = false;
    MyPosition.initialize(robot.mfr.getCurrentPosition(), robot.mfl.getCurrentPosition(), //initialize drivetrain tracker
                                                robot.mbl.getCurrentPosition(), this.robot);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(!robot.isTeleopDriveAuto){ //Manual robot control
            robot.movement_turn = gamepad1.right_stick_x;
            robot.movement_y = gamepad1.left_stick_y;
            robot.movement_x = gamepad1.left_stick_x;
        }

        if(robot.isTeleopDriveAuto){ // Automatic robot control
            if(RobotMovement2.followCurve(robot.testTeleopPath, Math.toRadians(90), false) == true){ //if target reached
                robot.isTeleopDriveAuto = false;                                  //by auto control, set back to manual control
            }
        }

        if(gamepad1.b && robot.autoDriveCooldown.time() >= 3){ // initializes automatic controls
            robot.isTeleopDriveAuto = true;
            robot.testTeleopPath.clear();
            robot.testTeleopPath.add(robot.startOfPath);
            robot.testTeleopPath.add(robot.testTeleopPathWaypoint1);
            robot.testTeleopPath.add(robot.testTeleopPathWaypoint2);
            RobotMovement2.initCurve();
            RobotMovement2.initForMove();
        }

        if(gamepad1.x){
            robot.isTeleopDriveAuto = false;
        }



        robot.ApplyMovement();//Updates drivetrain powers
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
