//package org.firstinspires.ftc.teamcode.operations;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.purepursuit.Path;
//import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
//import org.openftc.apriltag.AprilTagDetection;
//
//import java.util.ArrayList;
//
//@Autonomous
//@Config
//public class MirroredAutonomousOP extends LinearOpMode {
//    public Robot robot;
//
//    private XyhVector lastDestination = new XyhVector(-14, 102, 0); // prev -181 4 0
//    private int numFramesWithoutDetection = 0;
//    private final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
//    private final float DECIMATION_LOW = 2;
//    private final double THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
//    private final float DECIMATION_HIGH = 3;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        this.robot = new Robot(this);
//        robot.hardware.driveTrain.leftClaw.setPosition(0);
//
//        waitForStart();
//
//        ElapsedTime timer = new ElapsedTime();
//
//
//        this.robot.hardware.odometry.pose2d = new Pose2d(-67.3, 366, Rotation2d.fromDegrees(180));
//        this.robot.hardware.odometry.pos = new XyhVector(-67.3, 366, Math.toRadians(180));
//
//        while (opModeIsActive()) {
//            this.robot.hardware.odometry.update();
//
//            this.robot.hardware.driveTrain.leftLift.goToPosition(3500, 0.3);
//
//            movePath(new Path(
//                    new StartWaypoint(-67.3, 366),
//                    new EndWaypoint(
//                            -71, 329, 0, 0.65,
//                            0.5, 10, 2, 0.3
//                    )));
//
//            sleep(1000);
//            timer.reset();
//            int i = 0;
//
//            while (this.robot.hardware.objectDetected.equals("")) {
//                this.detectObjects(timer);
//
//                if (timer.seconds() > 2.0) {
//                    break;
//                }
//            }
//
//            sleep(4000);
//
//            if (timer.seconds() >= 2.0) {
//                movePath(new Path(
//                                new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
//                                new GeneralWaypoint(-73, 364, 0, 1,
//                                        0.5, 30),
//                                new GeneralWaypoint(-150, 364, 0, 1,
//                                        0.5, 30),
//                                new EndWaypoint(-150, 280, 0, 1,
//                                        0.5, 30, 2, 0.2)
//                        )
//                );
//
//                movePath(new Path(
//                        new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
//                        new EndWaypoint(-160, 279, 0, 0.65,
//                                0.5, 10, 1, 0.2)
//                ));
//
//                sleep(1000);
//                this.robot.hardware.driveTrain.leftClaw.setPosition(0.75);
//                sleep(1000);
//
//                movePath(new Path(
//                        new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
//                        new EndWaypoint(-158, 271, 0, 1,
//                                0.5, 30, 2, 0.2)
//                ));
//                movePath(new Path(
//                        new StartWaypoint(this.robot.hardware.odometry.pos.x, this.robot.hardware.odometry.pos.y),
//                        new GeneralWaypoint(-158, 271, 0, 1,
//                                0.5, 30),
//                        new GeneralWaypoint(-158, 256, 0, 1,
//                                0.5, 30),
//                        new EndWaypoint(this.lastDestination.x, this.lastDestination.y,
//                                0, 1, 0.5, 30, 2, 1)
//                ));
//                this.robot.hardware.driveTrain.leftLift.goToPosition(0, 0.3);
//                sleep(5000);
//
//                this.robot.hardware.outputReadings();
//                stop();
//            }
//        }
//    }
//
//    public void detectObjects(ElapsedTime timer) {
//        while(true) {
//            ArrayList<AprilTagDetection> detections = this.robot.hardware.detector.getDetectionsUpdate();
//
//            if (detections != null) {
//                if (detections.size() == 0) {
//                    numFramesWithoutDetection++;
//
//                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
//                        this.robot.hardware.detector.setDecimation(DECIMATION_LOW);
//                    }
//                } else {
//                    numFramesWithoutDetection = 0;
//
//                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
//                        this.robot.hardware.detector.setDecimation(DECIMATION_HIGH);
//                    }
//
//                    for (AprilTagDetection detection : detections) {
//                        this.telemetry.addData("Found thing: ", detection.id);
//                        this.telemetry.update();
//
//                        if (detection.id == 0) {
//                            lastDestination = new XyhVector(-14, 102, 0);
//                        } else if (detection.id == 3) {
//                            lastDestination = new XyhVector(-71, 102, 0);
//                        } else if (detection.id == 6) {
//                            lastDestination = new XyhVector(-160, 102, 0);;
//                        }
//                    }
//
//                    break;
//                }
//            }
//        }
//    }
//
//    public void movePath(Path path) throws InterruptedException {
//        path.init();
//        path.setWaypointTimeouts(5000);
//
//        while (!path.isFinished()) {
//            if (path.timedOut())
//                throw new InterruptedException("Timed out");
//
//            XyhVector position = this.robot.hardware.odometry.pos;
//
//            // return the motor speeds
//            double[] speeds = path.loop(position.x, position.y, position.h);
//
//            this.robot.telemetry.addData("object", this.robot.hardware.objectDetected);
//
//            this.robot.movement.strafe(-speeds[0], speeds[1], speeds[2]);
//
//            this.robot.hardware.odometry.update();
//            this.robot.hardware.outputReadings();
//        }
//    }
//}
