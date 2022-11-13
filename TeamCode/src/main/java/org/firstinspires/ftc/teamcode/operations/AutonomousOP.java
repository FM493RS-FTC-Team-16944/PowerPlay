package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.movement.ImageCommand;
import org.firstinspires.ftc.teamcode.movement.TrackingPathFinder;

import java.lang.reflect.InvocationTargetException;
import java.util.List;

@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    public Robot robot;
    private TrackingPathFinder trackingPathFinder;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new Robot(this);
        this.trackingPathFinder = new TrackingPathFinder(this);

        waitForStart();

        while(opModeIsActive()) {
            movePath(new Path(
                    new StartWaypoint(0, 0),
                    new EndWaypoint(
                            400, 0, 0, 0.5,
                            0.5, 30, 0.8, 1
                    )
            ));

            List<String> detectedObjects = this.robot.hardware.detector.getObjects();

            for (String label : detectedObjects) {
                try {
                    this.trackingPathFinder.invoke(label);
                } catch (InvocationTargetException | IllegalAccessException e) {
                    e.printStackTrace();
                }
            }

            this.robot.hardware.odometry.update();
            this.robot.telemetry.update();
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

    @ImageCommand(name="1 Bolt")
    public void boltPath() {
        this.telemetry.addData("Object", "Bolt");
    }

    @ImageCommand(name="2 Bulb")
    public void bulbPath() {
        this.telemetry.addData("Object", "Bulb");
    }

    @ImageCommand(name="3 Panel")
    public void panelPath() {
        this.telemetry.addData("Object", "Panel");
    }
}
