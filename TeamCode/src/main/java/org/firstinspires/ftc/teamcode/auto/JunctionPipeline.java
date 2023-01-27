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

// have JunctionPipeline return an x coordinate and a y coordinate in a list, maybe?

class JunctionPipeline extends OpenCvPipeline {

    double[] junctionCoords = new double[]{0, 0};

    public Mat processFrame(Mat input) {
        Scalar darkestJunctions = new Scalar(30, 80, 80);
        Scalar lightestJunctions = new Scalar(69, 100, 85);

        // apparently openCV likes BGR more than RGB, so convert our image from RGB to BGR
        Mat rawBGR = new Mat();
        Imgproc.cvtColor(input, rawBGR, Imgproc.COLOR_RGB2BGR);

        // Threshold image, turning it into binary (only black and white). Now openCV knows what to get the contour, or shape, of.
        Mat thresholded = new Mat();
        Core.inRange(rawBGR, darkestJunctions, lightestJunctions, thresholded);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint biggestContour = new MatOfPoint();

        // Idea: filter contours based on aspect ratio to only get tall objects here.

        // Get biggest contour
        for (MatOfPoint curContour : contours) {
            if (Imgproc.contourArea(curContour) > Imgproc.contourArea(biggestContour)) {
                biggestContour = curContour;
            }
        }

        Moments moments = Imgproc.moments(biggestContour);

        Point junctionPoint = new Point(moments.get_m10() / moments.get_m00(),
                moments.get_m01() / moments.get_m00());

        junctionCoords[0] = junctionPoint.x;
        junctionCoords[1] = junctionPoint.y;

        return thresholded;
    }

    public double[] getJunctionCoords() {return junctionCoords;}
}