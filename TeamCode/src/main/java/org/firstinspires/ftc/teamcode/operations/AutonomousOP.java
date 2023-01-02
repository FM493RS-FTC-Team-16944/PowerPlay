package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.openftc.apriltag.AprilTagDetection;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;


@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-88.4, 36.8, Math.toRadians(2700));

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-2.5,30.8,Math.toRadians(170)))
                .build();

        Trajectory destination1 = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(-10,11),Math.toRadians(180))
                .build();
        Trajectory destination2 = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(-10,35),Math.toRadians(0))
                .build();

        Trajectory destination3 = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(-10,69),Math.toRadians(180))
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