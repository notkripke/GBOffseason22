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

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
