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

class BasicPipelineTeleop extends OpenCvPipeline {

    Scalar darkestJunctions = new Scalar(0,0,0);
    Scalar lightestJunctions = new Scalar(360, 100, 100);

    public Mat processFrame(Mat input) {

        // apparently openCV likes BGR more than RGB, so convert our image from RGB to BGR
        Mat rawBGR = new Mat();
        Imgproc.cvtColor(input, rawBGR, Imgproc.COLOR_RGB2BGR);

        // Threshold image, turning it into binary (only black and white). Now openCV knows what to get the contour, or shape, of.
        Mat thresholded = new Mat();
        Core.inRange(rawBGR, lightestJunctions, darkestJunctions, thresholded);
        return thresholded;
    }

    public void setScalarValues(int scalar, int H, int S, int V) {
        Scalar tempScalar = new Scalar(H, S, V);
        switch (scalar) {
            case 0:
                darkestJunctions = tempScalar;
            case 1:
                lightestJunctions = tempScalar;
        }
    }
}