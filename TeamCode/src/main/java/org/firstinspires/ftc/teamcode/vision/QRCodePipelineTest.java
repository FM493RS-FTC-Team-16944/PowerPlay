package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

public class QRCodePipelineTest extends OpenCvPipeline {
    String data;
    boolean viewportPaused;

    @Override
    public Mat processFrame(Mat input) {
        QRCodeDetector decoder = new QRCodeDetector();

        Mat points = new Mat();
        this.data = decoder.detectAndDecode(input, points);

        if (!points.empty()) {
            System.out.println("Decoded data: " + data);

            for (int i = 0; i < points.cols(); i++) {
                Point pt1 = new Point(points.get(0, i));
                Point pt2 = new Point(points.get(0, (i + 1) % 4));
                Imgproc.line(input, pt1, pt2, new Scalar(255, 0, 0), 3);
            }
        }

        return input;
    }

    public String getLatestResult() {
        return data;
    }
}
