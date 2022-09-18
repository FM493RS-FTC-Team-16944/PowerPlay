package org.firstinspires.ftc.teamcode.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.geometry.CurvePoint;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;

import java.util.ArrayList;

public class PurePursuit {
    private final Odometry odometry;
    private final RobotMovement movement;

    PurePursuit(Odometry odometry, RobotMovement movement) {
        this.odometry = odometry;
        this.movement = movement;
    }

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        Pose2d position = this.odometry.getPosition();

        double distance = Math.hypot(x - position.getX(), y - position.getY());

        double absoluteAngleToTarget = Math.atan2(y - position.getY(), x - position.getX());

        // whats the difference between this aboslute and relative
        double relativeAngleToTarget = AngleUnit.normalizeRadians(absoluteAngleToTarget - (position.getHeading() - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToTarget) * distance;
        double relativeYToPoint = Math.sin(relativeAngleToTarget) * distance;

        double ratioDenominator = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);

        double movementXPower = (relativeXToPoint / ratioDenominator) * movementSpeed;
        double movementYPower = (relativeYToPoint / ratioDenominator) * movementSpeed;

        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + preferredAngle;

        double movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        // not sure if correct check video
        // this.movement.strafe(movementXPower, movementYPower, movementTurn);

        if (distance < 10) {
            movementTurn = 0;

            // this.movement.strafe(movementXPower, movementYPower, movementTurn);
        }
    }

    public ArrayList<XyhVector> lineCircleIntersection(XyhVector circleCenter, double radius,
                                                       XyhVector linePoint1, XyhVector linePoint2) {
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 3;
        }

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 3;
        }

        double m1 = linePoint2.y - linePoint1.y / (linePoint2.x - linePoint2.y);

        double quadraticA = 1.0 + Math.pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2));

        ArrayList<XyhVector> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticA))) / (2.0 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = Math.min(linePoint1.x, linePoint2.x);
            double maxX = Math.max(linePoint1.x, linePoint2.x);

            if(xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new XyhVector(xRoot1, yRoot1, 0));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new XyhVector(xRoot2, yRoot2, 0));
            }
        } catch (Exception e) {}

        return allPoints;
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, XyhVector robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size(); i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<XyhVector> intersections = lineCircleIntersection(robotLocation, followRadius,
                    startLine.toPoint(), endLine.toPoint());

            double closestAngle = Math.pow(10, 7);

            for(XyhVector thisIntersection : intersections) {
                Pose2d currentPos = this.odometry.getPosition();

                double angle = Math.atan2(thisIntersection.y - currentPos.getY(), thisIntersection.x - currentPos.getX());
                double deltaAngle = Math.abs(AngleUnit.normalizeRadians(angle - currentPos.getHeading()));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

        return followMe;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        Pose2d position = this.odometry.getPosition();

        XyhVector vector = new XyhVector(position.getX(), position.getY(), position.getHeading());

        CurvePoint followMe = getFollowPointPath(allPoints, vector, allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }
}
