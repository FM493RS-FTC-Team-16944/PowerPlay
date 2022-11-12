package org.firstinspires.ftc.teamcode.operations.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.movement.PurePursuit;
import org.firstinspires.ftc.teamcode.util.geometry.CurvePoint;

import java.util.ArrayList;

@Autonomous
public class PurePursuitTest extends LinearOpMode {
    PurePursuit purePursuit;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        this.purePursuit = new PurePursuit(
                robot.hardware.odometry1,
                robot.movement
        );

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        allPoints.add(
                new CurvePoint(
                        0, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0
                )
        );

        allPoints.add(
                new CurvePoint(
                        100, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0
                )
        );

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.hardware.odometry1.update();
            robot.hardware.odometry1.updateOdometryReadings();

            this.purePursuit.followCurve(allPoints, Math.toRadians(90));
        }
    }
}
