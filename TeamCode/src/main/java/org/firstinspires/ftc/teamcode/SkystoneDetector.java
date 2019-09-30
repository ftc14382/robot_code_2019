package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

public class SkystoneDetector extends DogeCVDetector {
    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for pre-processing and working with (blurring as an example)
    private Mat workingMatHsv = new Mat(); //
    private Mat maskYellow = new Mat(); // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by contours
    private Mat maskRgb    = new Mat(); // Used to display the mask
    public SkystoneDetectionState currentDetectionState;

    // This is our constructor. Call the constructor on our parent.
    public SkystoneDetector() {
        super();
        detectorName = "Skystone Detector"; // Set the detector name
        currentDetectionState = new SkystoneDetectionState();
    }

    //
    // This is where we do any detection that we want.
    // Think of this as being called over and over.
    // We constantly get new frames and we keep track of the last or last few detections.
    //
    // The input image is, oddly enough, named "input".
    // We can do anything that we want in the proces function as long as we return an image
    // that is the same size as input. We can put whatever we want on the image that we
    // return.
    //
    // Note: the input image is 480x640 in RGB format. We transpose it to
    // 640x480 while working with it to match up with the orientation of the camera.
    // We transpose the output back to 480x640 before returning it.
    @Override
    public Mat process(Mat input) {
        Size imageSize;
        Scalar colorRed   = new Scalar(0, 0, 255);
        int lineThickness = 2;
        int lineType      = Imgproc.LINE_8;
        Point lineStart;
        Point lineEnd;

        // Copy the input mat to our working mats, then release it for memory.
        // Note that the image that we get is portrait mode. The transpose
        // below makes it more intuitive to program: Image Y axis corresponds to
        // camera Y axis.
        // The transpose on displayMat is undone right before we return it.
        Core.transpose(input,displayMat);
        displayMat.copyTo(workingMat);
        input.release();

        // Create an HSV copy of workingMat and detect the yellow "Hue".
        Imgproc.cvtColor(workingMat, workingMatHsv, Imgproc.COLOR_RGB2HSV);
        Scalar lowerYellow = new Scalar(20, 50, 50);
        Scalar upperYellow = new Scalar(40, 255, 255);
        // The output of inRange is a "grayscale" image. A grayscale image
        // only has one number per pixel (usually called "Y"). Compare to
        // BGR which has three numbers per pixel (Blue, Green, and Red).
        // In fact, the output of inRange is an image in which all pixels are
        // either one or zero. This is sometimes called a "binary image" or
        // simply "a mask".
        Core.inRange(workingMatHsv, lowerYellow, upperYellow, maskYellow);

        // Now we take our grayscale maskYellow image and create an RGB image.
        Imgproc.cvtColor(maskYellow, displayMat, Imgproc.COLOR_GRAY2RGB);

        // This finds the contours in the yellowMask image.
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursYellowBig = new ArrayList<>();

        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint biggestContour;
        double area;
        double maxArea = 0.0;
        for (MatOfPoint c : contoursYellow) {
            area = Imgproc.contourArea(c);
            if (area > maxArea) {
                maxArea = area;
                biggestContour = c;
            }
            if (Imgproc.contourArea(c) > 300 ) {
                contoursYellowBig.add(c);
            }
        }

        // This draws the contours that we found onto displayMat in a greenish color.
        Imgproc.drawContours(displayMat, contoursYellowBig, -1, new Scalar(50, 230, 50), 2);

        // Get the size (width, height) of the image.
        imageSize = displayMat.size();

        // Draw a horizontal line through the center as an example
        lineStart = new Point(0, imageSize.height/2);
        lineEnd   = new Point(imageSize.width, imageSize.height/2);
        Imgproc.line(displayMat, lineStart, lineEnd, colorRed, lineThickness, lineType);

        // If we detect the Skystone block state, we update currentDetectionState.
        currentDetectionState.telemetry1 = "Nothing to see here.";

        // This gets displayMat back to the portrait mode that the rest of the pipeline is expecting.
        Core.transpose(displayMat,displayMat);
        return displayMat;
    }

     @Override
    public void useDefaults() {
        // We don't need to do anything here, but we have to define this method.
     }
}
