package org.firstinspires.ftc.teamcode.Teamcode.teamcode.FTCLibTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;

public class TestAuto extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();

        Pose2d p2_pos = drive.ItoMPose2d(80, 72, 0);

        drive.m_odometry.resetPosition(drive.ItoMPose2d(72, 72, 0), imu.getRotation2d());

        Waypoint p1 = new StartWaypoint(drive.m_odometry.getPoseMeters());

        Waypoint p2 = new GeneralWaypoint(p2_pos.getX(), p2_pos.getY(), p2_pos.getHeading(), 0,0);

        Path path = new Path(p1, p2);
        path.init();

        waitForStart();

        path.followPath(drive.d, drive.m_odometry);

        while (!isStopRequested()) {

        }
    }
}
