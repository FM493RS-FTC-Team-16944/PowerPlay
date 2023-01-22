package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.VerticalLiftPID;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    private MecanumDrive drive;

    AprilTagDetection detector = new AprilTagDetection();

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap);

        VerticalLiftPID zeroHeightPID;

        Pose2d startPose = new Pose2d(27, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        drive.outputOdomReadings(telemetry);


        Trajectory offWall = drive.trajectoryBuilder(startPose, true)
                .lineToConstantHeading(new Vector2d(32, -60),
                        MecanumDrive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
        Trajectory preTurnA = drive.trajectoryBuilder(offWall.end(), true)
                .lineToConstantHeading(new Vector2d(31, -1),
                        MecanumDrive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        Trajectory preTurnB = drive.trajectoryBuilder(preTurnA.end(), true)
                .lineToConstantHeading(new Vector2d(31, -18),
                        MecanumDrive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        Trajectory toCone = drive.trajectoryBuilder(preTurnB.end().plus(new Pose2d(0, 0, Math.toRadians(-165))))
                .lineToConstantHeading(new Vector2d(38, -10),
                        MecanumDrive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        Trajectory dropCone = drive.trajectoryBuilder(toCone.end())
                .lineToConstantHeading(new Vector2d(39, -6),
                        MecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        Trajectory backFromCone = drive.trajectoryBuilder(dropCone.end())
                .lineToConstantHeading(new Vector2d(38, -10),
                        MecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        Trajectory middleDestination = drive.trajectoryBuilder(backFromCone.end().plus(new Pose2d(0, 0, Math.toRadians(-15))))
                .lineToConstantHeading(new Vector2d(31, -10),
                        MecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();


        Trajectory midDestination = drive.trajectoryBuilder(middleDestination.end())
                .strafeLeft(3)
                .build();


        Trajectory leftDestination = drive.trajectoryBuilder(middleDestination.end())
                .strafeLeft(25)
                .build();

        Trajectory rightDestination = drive.trajectoryBuilder(middleDestination.end())
                .strafeRight(20)
                .build();

        Trajectory[] parkingSpots = new Trajectory[]{leftDestination,midDestination, rightDestination};
        int lastDestination = 1;


        waitForStart();


        drive.intake.hangingIntake();
            drive.intake.closeClaw();

           sleep(1000);

            AprilTagDetection detections = drive.detector.detectObjects();
            if(detections != null) {
                if (detections.id == 0) {
                    lastDestination = 0;
                } else if (detections.id == 3) {
                    lastDestination = 1;
                } else if (detections.id == 6) {
                    lastDestination = 2;
                }
                telemetry.addData("Detected Object", detections.id);
            }else{
                telemetry.addLine("No object found");
            }
            telemetry.addData("LastDestination", lastDestination);
            telemetry.update();

            drive.followTrajectory(offWall);
            drive.followTrajectory(preTurnA);
            drive.followTrajectory(preTurnB);
            drive.turn(Math.toRadians(-165));
            drive.followTrajectory(toCone);
            drive.followTrajectory(dropCone);


            this.telemetry.addLine("Launching Cone 1");
            drive.lift.verticalLiftEncoder.setPower(0.4);
            drive.lift.setVerticalLift(HIGH_SCORE_VERTICAL_LIFT_POSITION);
            sleep(3000);
            drive.lift.verticalLiftEncoder.setPower(0.6);
            drive.lift.setVerticalLift(NEUTRAL_VERTICAL_LIFT_POSITION);
            sleep(3000);

            drive.followTrajectory(backFromCone);
            drive.turn(Math.toRadians(-15));

            drive.followTrajectory(middleDestination);
            drive.followTrajectory(parkingSpots[lastDestination]);



            //drive.followTrajectory(parkingSpots[lastDestination]);
    }



}