//package org.firstinspires.ftc.teamcode.vision;
//
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class ShapePipeline extends OpenCvPipeline {
//    @Override
//    public void init(Mat input) {
//
//    }
//
//    @Override
//    public Mat processFrame(Mat input) {
//        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//
//        Mat processing = new Mat();
//        input.convertTo(processing, CvType.CV_32SC1);
//
//        Imgproc.findContours(processing, contours, new Mat(), Imgproc.RETR_FLOODFILL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        // Draw all the contours such that they are filled in.
//        Mat contourImg = new Mat(processing.size(), processing.type());
//        for (int i = 0; i < contours.size(); i++) {
//            Imgproc.drawContours(contourImg, contours, i, new Scalar(255, 255, 255), -1);
//        }
//
//        return processing;
//    }
//}