package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous
@Config
public class MirroredAutonomousOP extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, this.telemetry);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(180));

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0,0,Math.toRadians(0))) //fix
                .build();

        Trajectory destination1 = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(0,0),Math.toRadians(0))  //fix
                .build();
        Trajectory destination2 = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(0,0),Math.toRadians(0))
                .build();

        Trajectory destination3 = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(0,0),Math.toRadians(0))
                .build();

        Trajectory[] parkingSpots = new Trajectory[]{destination1,destination2,destination3};
        int lastDestination = 0;


        waitForStart();

        while (opModeIsActive()) {

            sleep(1000);

            AprilTagDetection detections = drive.detector.detectObjects();

            switch (detections.id) {
                case 0:
                    lastDestination = 0;
                case 3:
                    lastDestination = 1;
                case 6:
                    lastDestination = 2;
            }


            sleep(4000);

            drive.followTrajectory(forward);

            drive.followTrajectory(parkingSpots[lastDestination]);
        }
    }
}