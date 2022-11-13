package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.LiftMacro;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;

import java.util.List;

@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    public Robot robot;

    public XyhVector absolutePosition = new XyhVector(-83, 0, 0);
    private XyhVector lastDestination;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new Robot(this);

        LiftMacro liftMacroLeft = new LiftMacro(1900, this.robot.hardware.driveTrain, "leftLift");
        Thread lt0 = new Thread(liftMacroLeft);

        LiftMacro liftMacroRight = new LiftMacro(0, this.robot.hardware.driveTrain, "rightLift");
        Thread rt0 = new Thread(liftMacroRight);

        waitForStart();

        while(opModeIsActive()) {
            movePath(new Path(
                    new StartWaypoint(0, 0),
                    new InterruptWaypoint(
                            10 - this.absolutePosition.x, 0 - this.absolutePosition.y, 0, 0.5,
                            0.5, 30, 0.8, 1,
                            this::detectObjects
                    ),
                    new GeneralWaypoint(-144.5 - this.absolutePosition.x, 0 - this.absolutePosition.y),
                    new InterruptWaypoint(-144.5 - this.absolutePosition.x, 102 - this.absolutePosition.y, 0, 0.5,
                            0.5, 30, 0.8, 1,
                            lt0::start),
                    new GeneralWaypoint(-144.5 - this.absolutePosition.x, 75.5 - this.absolutePosition.y),
                    new InterruptWaypoint(-95 - this.absolutePosition.x, 75.5 - this.absolutePosition.y, Math.toRadians(90), 0.5,
                            0.5, 30, 0.8, 1,
                            () -> { this.robot.hardware.driveTrain.rightClaw.setPosition(0.75); this.robot.hardware.driveTrain.rightClaw.setPosition(0); }),
                    new GeneralWaypoint(-73 - this.absolutePosition.x, 75.5 - this.absolutePosition.y),
                    new GeneralWaypoint(-73 - this.absolutePosition.x, 135 - this.absolutePosition.y),
                    new GeneralWaypoint(-124 - this.absolutePosition.x, 135 - this.absolutePosition.y),
                    new InterruptWaypoint(-124 - this.absolutePosition.x, 160 - this.absolutePosition.y, 0, 0.5,
                            0.5, 30, 0.8 ,1,
                            rt0::start),
                    new GeneralWaypoint(-124 - this.absolutePosition.x, 135 - this.absolutePosition.y),
                    new EndWaypoint(this.lastDestination.x - this.absolutePosition.x, this.lastDestination.y - this.absolutePosition.y,
                            0, 0.5, 0.5, 30, 0.8, 1)
            ));

            this.robot.hardware.odometry.update();
            this.robot.telemetry.update();
            this.updateAbsolutePosition();
        }
    }

    public void detectObjects() {
        List<String> detectedObjects = this.robot.hardware.detector.getObjects();

        for (String label : detectedObjects) {
            switch (label) {
                case "1 Bolt":
                    lastDestination = new XyhVector(-14, 135, 0);
                    break;
                case "2 Bulb":
                    lastDestination = new XyhVector(-71, 135, 0);
                    break;
                case "3 Panel":
                    lastDestination = new XyhVector(-124, 135, 0);
                    break;
            }
        }
    }

    public void updateAbsolutePosition() {
        this.absolutePosition.x += this.robot.hardware.odometry.pose2d.getX();
        this.absolutePosition.y += this.robot.hardware.odometry.pose2d.getY();
        this.absolutePosition.h += this.robot.hardware.odometry.pose2d.getHeading();
    }

    public void movePath(Path path) throws InterruptedException {
        while (!path.isFinished()) {
            if (path.timedOut())
                throw new InterruptedException("Timed out");

            Pose2d position = this.robot.hardware.odometry.pose2d;

            // return the motor speeds
            double[] speeds = path.loop(position.getX(), position.getY(),
                    position.getHeading());

            this.robot.movement.strafe(speeds[0], speeds[1], speeds[2]);
            this.robot.hardware.odometry.update();
            this.updateAbsolutePosition();
        }
    }
}
