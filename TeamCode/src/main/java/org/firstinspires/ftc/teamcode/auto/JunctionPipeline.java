package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

// have JunctionPipeline return an x coordinate and a y coordinate in a list, maybe?

class JunctionPipeline extends OpenCvPipeline {
    public Mat processFrame (Mat input) {
        // apparently openCV likes BGR more than RGB
        Mat rawBGR = new Mat();
        Imgproc.cvtColor(input, rawBGR, Imgproc.COLOR_RGB2BGR);

        Mat rChannel = 
        return input;
    }
}