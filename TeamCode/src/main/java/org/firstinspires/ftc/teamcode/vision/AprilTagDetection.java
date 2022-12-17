package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class AprilTagDetection {
    private final HardwareMap hardwareMap;
    private final RobotHardware hardware;
    private OpenCvWebcam webcam;
    public AprilTagDetectionPipeline pipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public AprilTagDetection(RobotHardware hardware) {
        this.hardware = hardware;
        this.hardwareMap = hardware.hardwareMap;
        this.pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
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

    public ArrayList<org.openftc.apriltag.AprilTagDetection> getDetectionsUpdate() {
        return this.pipeline.getDetectionsUpdate();
    }

    public void setDecimation(float val) {
        this.pipeline.setDecimation(val);
    }
}
