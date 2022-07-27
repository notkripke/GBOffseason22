package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class GorillabotsCentral extends LinearOpMode {

    public Drivetrain drive;
    public RevIMU imu;

    public void initializeComponents(){

    drive = new Drivetrain();
    imu = new RevIMU(hardwareMap);
    imu.init();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
