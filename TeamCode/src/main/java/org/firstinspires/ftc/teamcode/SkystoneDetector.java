package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SkystoneDetector extends DogeCVDetector {
    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for pre-processing and working with (blurring as an example)
    private Mat maskYellow = new Mat(); // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by contours
    public SkystoneDetecionState currentDetectionState;

    // This is our constructor. Call the constructor on our parent.
    public GoldAlignDetector() {
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
    @Override
    public Mat process(Mat input) {
        Size imageSize;
        Scalar colorRed   = new Scalar(0, 0, 255);
        int lineThickness = 2;
        int lineType      = Imgproc.LINE_8;
        Point lineStart;
        Point lineEnd;
        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();

        // getAdjustedSize is defined on our parent, DogeCVDetector.
        // An openCV Size object has a 'height' and 'width' properties.
        imageSize = getAdjustedSize();

        // Start and end of a horizontal line through the center.
        lineStart = Point(0, imageSize.height/2);
        lineEnd   = point(imageSize.width, imageSize.height/2);

        Imgproc.line(displayMat, lineStart, lineEnd, colorRed, lineThickness, lineType);

        // If we detect the Skystone block state, we should create a new currentDetectionState
        // and fill it in.

        return displayMat;
    }
}
