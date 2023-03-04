package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous
@Config
public class ParkOnlyAutonomousOP extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, this.telemetry);

        Pose2d startPose = new Pose2d(32.5, -65, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        drive.outputOdomReadings(telemetry);

//        TrajectorySequence cyclePosition = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .splineTo(new Vector2d(35, -45), Math.toRadians(90))
//                .splineTo(new Vector2d(35, -23), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(39, -10, Math.toRadians(165.95)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(31.96, -5.29), Math.toRadians(90))
//                .build();

        TrajectorySequence cyclePosition = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(35, -45), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(35, -23, Math.toRadians(164)))
                // .splineToSplineHeading(new Pose2d(39, -10, Math.toRadians(165.95)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(31.46, -3.54), Math.toRadians(100))
                .build();

        TrajectorySequence parkingSpot1 = drive.trajectorySequenceBuilder(cyclePosition.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33, -12, Math.toRadians(180)), Math.toRadians(90))
                .forward(22)
                .build();

        TrajectorySequence parkingSpot2 = drive.trajectorySequenceBuilder(cyclePosition.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33, -12, Math.toRadians(180)), Math.toRadians(90))
                .build();

        TrajectorySequence parkingSpot3 = drive.trajectorySequenceBuilder(cyclePosition.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33, -12, Math.toRadians(180)), Math.toRadians(90))
                .forward(-22)
                .build();

        TrajectorySequence[] parkingSpots = {parkingSpot1, parkingSpot2, parkingSpot3};

        drive.lift.activateSlideSupport();
        drive.odometry.lowerOdometry();

        waitForStart();



        drive.intake.rotatedHangingIntake();
        drive.intake.openClaw();

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        packet.put("external heading velo", this.drive.getExternalHeadingVelocity());

        dashboard.sendTelemetryPacket(packet);


        AprilTagDetection detection = drive.detector.detectObjects();
        int destinationIndex = 0;

        if (detection != null) {
            if (detection.id == 3) {
                destinationIndex = 1;
            } else if (detection.id == 6) {
                destinationIndex = 2;
            }
        }

        drive.intake.rotatedHangingIntake();
        drive.lift.verticalLiftEncoder.setTargetPosition(0);
        drive.lift.horizontalSlide.setTargetPosition(0);

        drive.followTrajectorySequence(parkingSpots[destinationIndex]);

        PoseStorage.currentPos = drive.getPoseEstimate();
    }
}