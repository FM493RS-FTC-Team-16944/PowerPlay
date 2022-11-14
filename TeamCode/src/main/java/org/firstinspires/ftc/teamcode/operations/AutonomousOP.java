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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;

import java.util.List;

@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    public Robot robot;

    private XyhVector lastDestination = new XyhVector(-181, 4, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        this.robot = new Robot(this);

        this.robot.hardware.odometry.pose2d = new Pose2d(-67.3, 0, new Rotation2d());
        this.robot.hardware.odometry.pos = new XyhVector(-67.3, 0, 0);

        while (opModeIsActive()) {
            this.robot.hardware.odometry.update();

            this.robot.hardware.driveTrain.leftClaw.setPosition(0);
            this.robot.hardware.driveTrain.leftLift.goToPosition(3725, 0.3);

            movePath(new Path(
                    new StartWaypoint(-67.3, 0),
                    new EndWaypoint(
                            -70, 30, 0, 1,
                            0.5, 30, 2, 0.3
                    )));

            this.robot.hardware.telemetry.addData("finished going to path", "yes");
            this.robot.hardware.outputReadings();

            this.detectObjects();

            this.robot.hardware.telemetry.addData("finished vision going to nedxt path", "yes");
            this.robot.hardware.outputReadings();

            movePath(new Path(
                    new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
                    new GeneralWaypoint(-83, 2, 0, 1,
                            0.5, 30),
                    new GeneralWaypoint(-155, 2, 0, 1,
                            0.5, 30),
                    new GeneralWaypoint(-155, 77, 0, 1,
                            0.5, 30),
                    new EndWaypoint(-174.44, 77, 0, 1,
                            0.5, 30, 1, 0.2)
                    )
            );

            sleep(1000);
            this.robot.hardware.driveTrain.leftClaw.setPosition(0.75);
            sleep(1000);


            movePath(new Path(
                    new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
                    new GeneralWaypoint(-160, 77, 0, 1,
                            0.5, 30),
                    new GeneralWaypoint(-160, 85, 0, 1,
                            0.5, 30),
                    new EndWaypoint(this.lastDestination.x, this.lastDestination.y,
                            0, 1, 0.5, 30, 2, 1)
            ));
            this.robot.hardware.driveTrain.leftLift.goToPosition(0, 0.3);
            sleep(2000);

            this.robot.hardware.outputReadings();
        }
    }

    public void detectObjects() {
        List<String> detectedObjects = this.robot.hardware.detector.getObjects();

        for (String label : detectedObjects) {
            switch (label) {
                case "SQUARE":
                    lastDestination = new XyhVector(-14, 110, 0);
                    this.telemetry.addData("Object Recognized: ", "SQUARE");
                    break;
                case "CIRCLE":
                    lastDestination = new XyhVector(-71, 110, 0);
                    this.telemetry.addData("Object Recognized: ", "CIRCLE");
                    break;
                case "TRIANGLE":
                    lastDestination = new XyhVector(-160, 110, 0);
                    this.telemetry.addData("Object Recognized: ", "TRIANGLE");
                    break;
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

            this.robot.telemetry.addData("Speed 0", speeds[0]);
            this.robot.telemetry.addData("Speed 1", speeds[1]);
            this.robot.telemetry.addData("Speed 2", speeds[2]);

            this.robot.telemetry.addData("Is Path Finished?",path.isFinished());

            this.robot.movement.strafe(-speeds[0], speeds[1], speeds[2]);

            this.robot.hardware.odometry.update();
            this.robot.hardware.outputReadings();
        }
    }
}
