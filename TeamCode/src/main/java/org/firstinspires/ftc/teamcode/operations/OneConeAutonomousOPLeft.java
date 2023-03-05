package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.RANGE_OF_UNSAFE_VERTICAL_LIFT;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.TILT_THRESHOLD;

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
public class OneConeAutonomousOPLeft extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, this.telemetry);

        Pose2d startPose = new Pose2d(32.5, 65, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        drive.outputOdomReadings(telemetry);

//        TrajectorySequence cyclePosition = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .splineTo(new Vector2d(35, -45), Math.toRadians(90))
//                .splineTo(new Vector2d(35, -23), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(39, -10, Math.toRadians(165.95)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(31.96, -5.29), Math.toRadians(90))
//                .build();

        TrajectorySequence loadFromStartPosition = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(35, 45), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(35, 23, Math.toRadians(180)))
                // .splineToSplineHeading(new Pose2d(39, -10, Math.toRadians(165.95)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(31, 14.25), Math.toRadians(100))
                .build();

        TrajectorySequence polePosition = drive.trajectorySequenceBuilder(loadFromStartPosition.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(5.65, 22, Math.toRadians(224)))
                .build();

        TrajectorySequence loadPosition = drive.trajectorySequenceBuilder(polePosition.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(31, 14.25, Math.toRadians(180)))
                .build();

        TrajectorySequence parkingSpot3 = drive.trajectorySequenceBuilder(loadPosition.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33, 12, Math.toRadians(90)), Math.toRadians(90))
                .strafeLeft(22)
                .build();

        TrajectorySequence parkingSpot2 = drive.trajectorySequenceBuilder(loadPosition.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33, 12, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence parkingSpot1 = drive.trajectorySequenceBuilder(loadPosition.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33, 12, Math.toRadians(90)), Math.toRadians(90))
                .strafeLeft(-23)
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

        drive.followTrajectorySequence(loadFromStartPosition);
        drive.followTrajectorySequence(polePosition);

        drive.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION);
        drive.intake.groundIntake(0);

        boolean tipping = false;

        while (true) {
            if (drive.lift.getVerticalLiftPosition() <= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION + 6 &&
                    drive.lift.getVerticalLiftPosition() >= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - 6)
                break;
        }

        drive.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);

        while (true) {
            if (Math.abs(drive.imu.getAngularOrientation().secondAngle) > TILT_THRESHOLD ||
                    Math.abs(drive.imu.getAngularOrientation().thirdAngle) > TILT_THRESHOLD) {
                this.drive.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION);
                tipping = true;
                break;
            } else if (drive.lift.getVerticalLiftPosition() < ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - RANGE_OF_UNSAFE_VERTICAL_LIFT) {
                break;
            }
        }

        drive.followTrajectorySequence(loadPosition);

        drive.intake.rotatedHangingIntake();
        drive.lift.verticalLiftEncoder.setTargetPosition(0);
        drive.lift.horizontalSlide.setTargetPosition(0);
        drive.followTrajectorySequence(parkingSpots[destinationIndex]);

        drive.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);


        PoseStorage.currentPos = drive.getPoseEstimate();
    }
}