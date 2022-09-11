package org.firstinspires.ftc.teamcode.vision;

public class ObjectDetector {
    
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "ball",
            "block",
            "duck"
    };

    private static final String VUFORIA_KEY =
            "AdwpsnD/////AAABmXLkfWRnc0QfuiNxVS6Yh4tOMfYBWgTqMB9KcHBB+YRPnbQLwETIZbsWLSUNI8SoPBEt3Sbi39RHPSQiGerufWT8VnW6WP0iT+tpJfcTdKRYKNa5k9ZQVC4eRJ4ROyersNVLlixPTu6Se6aTmlOhkWGxlSQawoPwJbrtvhSS9YKB1pr95zuC8uKz4luBSgaYU/bVSCgvE7oH2Gzlipi9TQCCp3OoW+kVogZQHxiGlzpGUhGGNC4gH/n4iB5KhAMcersPPedQgRr8hTGLmYIPFC5M6sTxhYPg4CWMyMKSg5kPKUr0uEh9SM5ZoxLdIEw9iUMSkmDRytYmob5A8v1vOI6J9FFHmKvlGFvSGkTNTRMC";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    private final HardwareMap hardware;

    ObjectDetection(RobotHardware hardware) {
        this.hardware = hardware.hardwareMap;
    }

    public void setupDetection() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }
    }


    public Recognition getNearestObject() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        return updatedRecognitions.get(0);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
        CameraDevice.getInstance().setField("zoom", "19");
    }


    private void initTfod() {
        int tfodMonitorViewId = hardware.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardware.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
