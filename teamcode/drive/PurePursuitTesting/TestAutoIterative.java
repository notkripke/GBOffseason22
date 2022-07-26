/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.CurvePoint;

import java.util.ArrayList;


@Autonomous(name="TestAutoIterative", group="auto")
public class TestAutoIterative extends OpMode
{
    public String program_stage = "start";

    Robot robot = new Robot(hardwareMap, telemetry);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {




        double x = 0;
        double y = 0;

        MyPosition.setPosition(100, 100, Math.toRadians(0)); //sets starting position like roadrunner

        ArrayList<CurvePoint> path1 = new ArrayList<>();

        path1.add(new CurvePoint(0,0,0,0,0,0,
                                0,0));

        path1.add(new CurvePoint(140,30,.7, .8, 5, 6, 2,2));

        telemetry.addData("Status", "Initialized");
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //RUNS PATH FOLLOWING THROUGH PURE PURSUIT
        ArrayList<CurvePoint> path1 = new ArrayList<>();
        path1.add(new CurvePoint(0,0,0,0,0,0,
                0,0));
        path1.add(new CurvePoint(140,30,.7, .8, 5, 6, 2,2));

        while(RobotMovement2.followCurve(path1,0,false) != true){
            //can do robot functions in here
            //
            //robot.updateOdometry();
            //
            robot.ApplyMovement();//Applies drivetrain powers
            //


            telemetry.addLine("following path 1");
            telemetry.update();
        }
        telemetry.addLine("done following path1");
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
