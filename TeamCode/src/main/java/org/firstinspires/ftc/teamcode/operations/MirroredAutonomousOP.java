package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;

import java.util.ArrayList;
import java.util.List;

@Autonomous
@Config
public class MirroredAutonomousOP extends LinearOpMode {
    public Robot robot;

    private XyhVector lastDestination = new XyhVector(-14, 102, 0); // prev -181 4 0

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new Robot(this);
        robot.hardware.driveTrain.leftClaw.setPosition(0);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();


        this.robot.hardware.odometry.pose2d = new Pose2d(-67.3, 366, Rotation2d.fromDegrees(180));
        this.robot.hardware.odometry.pos = new XyhVector(-67.3, 366, Math.toRadians(180));

        while (opModeIsActive()) {
            this.robot.hardware.odometry.update();

            this.robot.hardware.driveTrain.leftLift.goToPosition(3500, 0.3);

            movePath(new Path(
                    new StartWaypoint(-67.3, 366),
                    new EndWaypoint(
                            -71, 329, 0, 0.65,
                            0.5, 10, 2, 0.3
                    )));

            sleep(1000);
            timer.reset();
            int i = 0;

            while(this.robot.hardware.objectDetected.equals("")) {
                this.detectObjects(timer);

                if(timer.seconds() > 2.0) {
                    break;
                }
            }

            sleep(4000);

            if(timer.seconds() >= 2.0) {
                movePath(new Path(
                                new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
                                new GeneralWaypoint(-73, 364, 0, 1,
                                        0.5, 30),
                                new GeneralWaypoint(-150, 364, 0, 1,
                                        0.5, 30),
                                new EndWaypoint(-150, 280, 0, 1,
                                0.5, 30, 2, 0.2)
                        )
                );

                movePath(new Path(
                        new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
                        new EndWaypoint(-160, 279, 0, 0.65,
                                0.5, 10, 1, 0.2)
                ));

                sleep(1000);
                this.robot.hardware.driveTrain.leftClaw.setPosition(0.75);
                sleep(1000);

                movePath(new Path(
                        new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
                        new EndWaypoint(-158, 271, 0, 1,
                                0.5, 30, 2, 0.2)
                ));
                movePath(new Path(
                        new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
                        new GeneralWaypoint(-158, 271, 0, 1,
                                0.5, 30),
                        new GeneralWaypoint(-158, 256, 0, 1,
                                0.5, 30),
                        new EndWaypoint(this.lastDestination.x, this.lastDestination.y,
                                0, 1, 0.5, 30, 2, 1)
                ));
                this.robot.hardware.driveTrain.leftLift.goToPosition(0, 0.3);
                sleep(5000);

                this.robot.hardware.outputReadings();
                stop();
            }
        }
    }

    public void detectObjects(ElapsedTime timer) {
        List<String> detectedObjects = new ArrayList<>();
        detectedObjects.add(" ");
        telemetry.addData("Timer:", timer.seconds());
        if(timer.seconds() < 3.5 || detectedObjects.size() == 0) {
            detectedObjects.add(this.robot.hardware.detector.getLatestResult());
        }else {
            sleep(1000);
            for (String label : detectedObjects) {
                switch (label) {
                    case "SQUARE":
                        lastDestination = new XyhVector(-14, 264, 0);
                        this.telemetry.addData("Object Recognized: ", "SQUARE");
                        robot.hardware.objectDetected = "SQUARE";
                        break;
                    case "CIRCLE":
                        lastDestination = new XyhVector(-71, 264, 0);
                        this.telemetry.addData("Object Recognized: ", "CIRCLE");
                        robot.hardware.objectDetected = "CIRCLE";
                        break;
                    case "TRIANGLE":
                        lastDestination = new XyhVector(-160, 264, 0);
                        this.telemetry.addData("Object Recognized: ", "TRIANGLE");
                        robot.hardware.objectDetected = "TRIANGLE";
                        break;
                }
            }
        }
        this.telemetry.update();


    }

    public void movePath(Path path) throws InterruptedException {
        path.init();
        path.setWaypointTimeouts(5000);

        while (!path.isFinished()) {
            if (path.timedOut())
                throw new InterruptedException("Timed out");

            XyhVector position = this.robot.hardware.odometry.pos;

            // return the motor speeds
            double[] speeds = path.loop(position.x, position.y, position.h);

            this.robot.telemetry.addData("object", this.robot.hardware.objectDetected);

            this.robot.movement.strafe(-speeds[0], speeds[1], speeds[2]);

            this.robot.hardware.odometry.update();
            this.robot.hardware.outputReadings();
        }
    }
}
