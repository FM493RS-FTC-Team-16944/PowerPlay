package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class AprilTagDetector {
    private final HardwareMap hardwareMap;
    private final RobotHardware hardware;
    private OpenCvWebcam webcam;
    public AprilTagPipeline pipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 15;
    float DECIMATION_LOW = 2;
    double THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    float DECIMATION_HIGH = 3;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public AprilTagDetector(RobotHardware hardware) {
        this.hardware = hardware;
        this.hardwareMap = hardware.hardwareMap;
        this.pipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);
    }

    public void initDetector() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(this.pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                hardware.telemetry.addData("error code", errorCode);
            }
        });
    }

    public AprilTagDetection detectObjects() {
        int failed = 0;
        int numFramesWithoutDetection = 0;

        while(true) {
            ArrayList<AprilTagDetection> detections = this.pipeline.getDetectionsUpdate();

            if (detections != null) {
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    if(failed > 75) {
                        break;
                    }

                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        this.pipeline.setDecimation(DECIMATION_LOW);
                    }

                    failed++;
                } else {
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        this.pipeline.setDecimation(DECIMATION_HIGH);
                    }

                    return detections.get(0);
                }
            }
        }

        return null;
    }
}