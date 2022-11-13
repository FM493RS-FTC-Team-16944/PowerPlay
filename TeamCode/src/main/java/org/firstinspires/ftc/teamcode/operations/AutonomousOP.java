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
                    new StartWaypoint(-83, 0),
                    new InterruptWaypoint(
                            10, 0, 0, 0.5,
                            0.5, 30, 0.8, 1,
                            this::detectObjects
                    ),
                    new GeneralWaypoint(-144.5, 0),
                    new InterruptWaypoint(-144.5, 102, 0, 0.5,
                            0.5, 30, 0.8, 1,
                            lt0::start),
                    new GeneralWaypoint(-144.5, 75.5),
                    new InterruptWaypoint(-95, 75.5, Math.toRadians(90), 0.5,
                            0.5, 30, 0.8, 1,
                            () -> { this.robot.hardware.driveTrain.rightClaw.setPosition(0.75); this.robot.hardware.driveTrain.rightClaw.setPosition(0); }),
                    new GeneralWaypoint(-73, 75.5),
                    new GeneralWaypoint(-73, 135),
                    new GeneralWaypoint(-124, 135),
                    new InterruptWaypoint(-124, 160, 0, 0.5,
                            0.5, 30, 0.8 ,1,
                            rt0::start),
                    new GeneralWaypoint(-124, 135),
                    new EndWaypoint(this.lastDestination.x, this.lastDestination.y,
                            0, 0.5, 0.5, 30, 0.8, 1)
            ));

            this.robot.hardware.odometry.update();
            this.robot.telemetry.update();
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
        }
    }
}
