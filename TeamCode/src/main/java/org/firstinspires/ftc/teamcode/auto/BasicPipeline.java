package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

// have JunctionPipeline return an x coordinate and a y coordinate in a list, maybe?

public class BasicPipeline extends OpenCvPipeline {

    Scalar darkestJunctions = new Scalar(15, 128, 128);
    Scalar lightestJunctions = new Scalar(60, 255, 255);
    Mat rawHSV = new Mat();
    Mat blurredHSV = new Mat();
    Mat thresholded = new Mat();
    List<MatOfPoint> contoursAttr = new ArrayList<>();
    Point junctionPointAttr = new Point();
    double bigJunctionSizeAttr = 0;

    public Mat processFrame(Mat input) {
        // Convert image to HSV
        Imgproc.cvtColor(input, rawHSV, Imgproc.COLOR_RGB2HSV);

        // Blur image to lessen noise
        Imgproc.GaussianBlur(rawHSV, blurredHSV, new Size(15, 15), 0); // increase?

        // Threshold image, turning it into binary (only black and white). Now openCV knows what to get the contour, or shape, of.
        Core.inRange(blurredHSV, darkestJunctions, lightestJunctions, thresholded);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        contoursAttr = contours;

        if (!contours.isEmpty())
        {
            MatOfPoint biggestContour = contours.get(0);//new MatOfPoint();

            for (MatOfPoint curContour : contours) {
                if (Imgproc.contourArea(curContour) > Imgproc.contourArea(biggestContour)) {
                    biggestContour = curContour;
                }
            }

            Moments moments = Imgproc.moments(biggestContour);
            Point junctionPoint = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());


            bigJunctionSizeAttr = Imgproc.contourArea(biggestContour);
            junctionPointAttr = junctionPoint;
        }


        // List<Point> junctionPoints = new ArrayList<>();

        // for (MatOfPoint curContour : contours) {
        //     Moments moments = Imgproc.moments(curContour);
        //     junctionPoints.add(new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00()));
        // }

        // junctionPointsAttr = junctionPoints; monkey haha lol

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 3);

        return input;
    }

    public double getBigJunctionSizeAttr() {
        return bigJunctionSizeAttr;
    }
    public int getContourQuantity() {
        return contoursAttr.size();
    }
    public Point getJunctionPoint() {
        return junctionPointAttr;
    }
}