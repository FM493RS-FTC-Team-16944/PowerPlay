package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;

import android.graphics.Bitmap;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class QRCodePipeline extends OpenCvPipeline {
    private final Telemetry telemetry;
    OpenCvWebcam webcam;
    String data;

    boolean viewportPaused;

    public QRCodePipeline(OpenCvWebcam webcam, Telemetry telemetry) {
        this.webcam = webcam;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat grayMat = input.clone();

        Imgproc.cvtColor(input, grayMat, COLOR_BGR2HSV);
        Imgproc.threshold(grayMat, grayMat, 127, 255, THRESH_BINARY);

        List<MatOfPoint> matOfPoints = new ArrayList<>();
        Imgproc.findContours(grayMat, matOfPoints, new Mat(), RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (MatOfPoint matOfPoint: matOfPoints) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f matOfPoint2f = new MatOfPoint2f();

            matOfPoint.convertTo(matOfPoint2f, CV_32F);

            Imgproc.approxPolyDP(matOfPoint2f, approxCurve, 0.1 * Imgproc.arcLength(matOfPoint2f, true), true);

            ArrayList<MatOfPoint> list = new ArrayList<>();
            list.add(matOfPoint);

            Imgproc.drawContours(input, list, 0, new Scalar(0, 0, 255), 0);

            if(approxCurve.toArray().length == 3) {
                this.telemetry.addData("Shape", "3 sides");
            } else if(approxCurve.toArray().length == 4) {
                this.telemetry.addData("Shape", "4 sides");
            } else if(approxCurve.toArray().length == 5) {
                this.telemetry.addData("Shape", "5 sides");
            }

            this.telemetry.update();
        }

        return input;
    }

    @Override
    public void onViewportTapped()
    {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }

    public String getLatestResult() {
        return data;
    }
}
