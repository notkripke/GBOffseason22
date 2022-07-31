package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class GorillabotsCentral extends LinearOpMode {

    public Drivetrain drive;
    public RevIMU imu;

    public void initializeComponents(){

    drive = new Drivetrain();
    //drive.d.driveRobotCentric(drive.movement_x, drive.movement_y, drive.movement_turn); DRIVE.SETWEIGHTEDPOWERS();
    imu = new RevIMU(hardwareMap);
    imu.init();
    }

    /**
     * Takes typical Gamepad1 inputs and maps drivetrain powers accordingly.
     * Use drive.ApplyMovement() after this to convert drivetrain powers into individula motor powers
     */
    public void setDriveToController(){
        drive.movement_turn = gamepad1.right_stick_x;
        drive.movement_y = gamepad1.left_stick_y;
        drive.movement_x = gamepad1.left_stick_x;
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
