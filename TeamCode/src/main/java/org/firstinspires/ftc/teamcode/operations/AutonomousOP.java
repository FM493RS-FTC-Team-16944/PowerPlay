package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;




@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {


    private XyhVector lastDestination = new XyhVector(-14, 102, 0); // prev -181 4 0
    private int numFramesWithoutDetection = 0;
    private final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    private final float DECIMATION_LOW = 2;
    private final double THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    private final float DECIMATION_HIGH = 3;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .strafeRight(10)
                .forward(5)
                .build();

        Trajectory lastDestination;



        waitForStart();

        while (opModeIsActive()) {

            sleep(1000);

            this.detectObjects();

            lastDestination = drive.trajectoryBuilder(forward.end())
                    .splineToLinearHeading(new Pose2d(this.lastDestination.x,this.lastDestination.y,Math.toRadians(this.lastDestination.h)), Math.toRadians(0))
                    .build();

            sleep(4000);

            drive.followTrajectory(forward);

            drive.followTrajectory(lastDestination);


        }
    }



    public void detectObjects() {
        while (true) {
            ArrayList<AprilTagDetection> detections = this.robot.hardware.detector.getDetectionsUpdate();

            if (detections != null) {
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        this.robot.hardware.detector.setDecimation(DECIMATION_LOW);
                        this.telemetry.addData("lowered decimation", "lol");
                        this.telemetry.update();
                    }
                } else {
                    numFramesWithoutDetection = 0;

                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        this.robot.hardware.detector.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        this.telemetry.addData("Found thing: ", detection.id);
                        this.telemetry.update();

                        if (detection.id == 0) {
                            lastDestination = new XyhVector(-14, 102, 0);
                        } else if (detection.id == 3) {
                            lastDestination = new XyhVector(-71, 102, 0);
                        } else if (detection.id == 6) {
                            lastDestination = new XyhVector(-160, 102, 0);
                            ;
                        }
                    }

                    break;
                }
            }
        }
    }
}