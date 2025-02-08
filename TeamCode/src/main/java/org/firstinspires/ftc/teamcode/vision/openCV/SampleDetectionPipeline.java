package org.firstinspires.ftc.teamcode.vision.openCV;

import static org.firstinspires.ftc.vision.opencv.ColorSpace.YCrCb;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;


public class SampleDetectionPipeline extends OpenCvPipeline {
    boolean viewportPaused;

    double[] latestResults = {-999, -999};

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();


    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        inputToCb(input);


        // Find ~3 closest areas of high yellow intensity, then look at those areas and determine what orientation the samples within them are in,
        // then find the most optimally orientated one and get its field coordinates and rotation, and store them to the latestResults variable


        return Cb;
    }

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

//    public Vector2 getLatestResult() {
//        return latestResult;
//    }
}
