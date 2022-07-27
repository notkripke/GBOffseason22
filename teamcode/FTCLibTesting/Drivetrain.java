package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.os.SystemClock;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    public static Pose2d robot_pos = new Pose2d();

    Motor mfl, mfr, mbl, mbr;
    MecanumDrive d = new MecanumDrive(
            mfl = new Motor(hardwareMap, "frontLeft"),
            mfr = new Motor(hardwareMap, "frontRight"),
            mbl = new Motor(hardwareMap, "backLeft"),
            mbr = new Motor(hardwareMap, "backRight")
    );

    RevIMU imu = new RevIMU(hardwareMap);
    /**
     * This will accept a user-given position in inches and degrees
     * @param x in degrees
     * @param y in degrees
     * @param heading in degrees
     * @return a new Pose2d in meters and radians, to be used by the odometry
     */

    public Pose2d ItoMPose2d(double x, double y, double heading){
        double adj_x = x * 39.37;
        double adj_y = y * 39.37;
        Rotation2d adj_heading = new Rotation2d(Math.toRadians(heading));

        return new Pose2d(adj_x, adj_y,adj_heading);
    }

    Translation2d mfl_location = new Translation2d(0.1778, 0.1778);
    Translation2d mfr_location = new Translation2d(0.1778, -0.1778);
    Translation2d mbl_location = new Translation2d(-0.1778, 0.1778);
    Translation2d mbr_location = new Translation2d(-0.1778, -0.1778);

    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            mfl_location, mfl_location, mbl_location, mbr_location
    );

    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
            m_kinematics, imu.getRotation2d(), ItoMPose2d(72, 72, 0)
    );

    public void updateOdo(){
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                mfl.getRate(), mfr.getRate(), mbl.getRate(), mbr.getRate()
        );

        Rotation2d gyroAngle = Rotation2d.fromDegrees(imu.getHeading());

        robot_pos = m_odometry.updateWithTime(SystemClock.uptimeMillis() / 1000, gyroAngle, wheelSpeeds);
    }
}
