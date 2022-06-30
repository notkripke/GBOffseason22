package org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting;

import static org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MathFunctions.*;

import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.*;

import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.Robot;

import org.firstinspires.ftc.teamcode.Teamcode.teamcode.drive.PurePursuitTesting.MathFunctions;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement{

/*
    public static  void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){

        ArrayList<CurvePoint> pathExtended = (ArrayList<CurvePoint>) allPoints.clone();

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robot_pos_x, robot_pos_y), allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i ++){
            CurvePoint startLine = pathPoints.get(1);
            CurvePoint endLine = pathPoints.get(i + 1);

            //ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            /*double closestAngle = 10000000;

            for (Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - robot_pos_y, thisIntersection.x = robot_pos_x);
                double deltaAngle = Math.abs(MyPosition.AngleWrap(angle = robot_pos_r));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

        }
        return  followMe;
    }






    //to be fixed once ready for real trial. will be held in GorillabotsCentral-esque class

    public static double robot_pos_x = 0;

    public static double robot_pos_y = 0;

    public static double robot_pos_r = Math.toRadians(0);

    public static double power_x; //Used for the movement stuff. This will be subbed into the drive.setWeightedDrivePower() roadrunner function.
    public static double power_y;
    public static double power_r;


    //-----------------------------------------------------------------------------------


    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){

        double distanceToTarget = Math.hypot(x-robot_pos_x, y-robot_pos_y);

        double absoluteAngleToTarget = Math.atan2(y-robot_pos_y, x-robot_pos_x);

        double relativeAngleToPoint = MyPosition.AngleWrap(absoluteAngleToTarget - (robot_pos_r - Math.toRadians(90)));  // for this example, 90 will be forwards

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        power_x = movementXPower * movementSpeed;
        power_y = movementYPower * movementSpeed;
        power_r = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget < 10){
            power_r = 0;
        }

    }*/
}
