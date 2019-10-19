package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetectorCrop extends DogeCVDetector {
    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for pre-processing and working with (blurring as an example)
    private Mat workingMatHsv = new Mat(); //
    private Mat maskYellow = new Mat(); // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by contours
    private Mat maskRgb    = new Mat(); // Used to display the mask
    public SkystoneDetectionState currentDetectionState;


    // This is our constructor. Call the constructor on our parent.
    public SkystoneDetectorCrop() {
        super();
        detectorName = "Skystone Detector Crop"; // Set the detector name
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

        int cropX = 160;
        int cropY = 0;
        int cropWidth = 100;
        int cropHeight = 640;
        Rect rectCrop = new Rect(cropX, cropY, cropWidth, cropHeight);//look at https://stackoverflow.com/questions/35666255/get-a-sub-image-using-opencv-java
        Mat reverseMask = new Mat();
        Core.bitwise_not(maskYellow, reverseMask);
        Mat cropMask = new Mat(reverseMask, rectCrop);

        // Now we take our grayscale maskYellow image and create an RGB image.
        Imgproc.cvtColor(reverseMask, displayMat, Imgproc.COLOR_GRAY2RGB);

        Mat displayCrop = new Mat(displayMat, rectCrop);

        // This finds the contours in the yellowMask image.
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursYellowBig = new ArrayList<>();
        List<MatOfPoint> greatestContour = new ArrayList<>();
        List<MatOfPoint> ratioContour = new ArrayList<>();


        Point topLeft = new Point(cropX, cropY);
        Point topRight = new Point(cropX, cropY+cropHeight);
        Point bottomRight = new Point(cropX+cropWidth, cropY+cropHeight);
        Point bottomLeft = new Point(cropX+cropWidth, cropY);

        Point circleCenter = new Point(0,0);

        double areaCrop;
        double maxAreaCrop = 0.0;
        Imgproc.findContours(cropMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint c : contoursYellow) {
            areaCrop = Imgproc.contourArea(c);//Detect biggest yellow area
            if (areaCrop > maxAreaCrop) {
                maxAreaCrop = areaCrop;
                greatestContour.clear();
                greatestContour.add(c);

                circleCenter = new Point((int)(Imgproc.boundingRect(c).x + Imgproc.boundingRect(c).width/2), (int)(Imgproc.boundingRect(c).y + Imgproc.boundingRect(c).height/2));
            }
        }
        Imgproc.circle(displayCrop, circleCenter, 3, new Scalar(250, 10,10), -1);

        if(false) {
        int numbBlocks = 0;
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint biggestContour;

            double area;
            double maxArea = 0.0;
            for (MatOfPoint c : contoursYellow) {
                area = Imgproc.contourArea(c);//Detect biggest yellow area
                if (area > maxArea) {
                    maxArea = area;
                    biggestContour = c;
                    greatestContour.clear();
                    greatestContour.add(biggestContour);
                }
                if (Imgproc.contourArea(c) > 500) {//Create seperate array for possible skystones
                    contoursYellowBig.add(c);
                }
            }
            double ratio = 0.0;
            double maxRatio = -3.0;
            for (MatOfPoint c : contoursYellowBig) {
                ratio = Imgproc.boundingRect(c).height / Imgproc.boundingRect(c).width;//Calculate ratio
                if (-Math.abs(ratio - 1.6) > maxRatio) {//Detect skystone with best height:width ratio
                    maxRatio = ratio;
                    ratioContour.clear();//Make sure we only have one selected countour
                    ratioContour.add(c);


                    if (false) {
                        numbBlocks = (int) (ratio / 1.6);//find the number of blocks inside the box you're outlining
                        if (numbBlocks > 1) {//Make sure we actually need the array
                            int blockLength = (Imgproc.boundingRect(c).height / numbBlocks) + 1;
                            Point[] topPoints = new Point[numbBlocks - 1];//create array with number of points equal to the lines between blocks
                            for (int i = 1; i < numbBlocks; i++) {//Find end points on top of the blocks
                                topPoints[i - 1] = new Point(Imgproc.boundingRect(c).x, Imgproc.boundingRect(c).y + blockLength * i);
                            }
                            Point[] bottomPoints = new Point[numbBlocks - 1];
                            for (int i = 1; i < numbBlocks; i++) {//Find end points bellow the blocks
                                bottomPoints[i - 1] = new Point(Imgproc.boundingRect(c).x + Imgproc.boundingRect(c).width, Imgproc.boundingRect(c).y + blockLength * i);
                            }
                            for (int i = 0; i < numbBlocks - 1; i++) {//Draw lines between blocks
                                Imgproc.line(displayMat, topPoints[i], bottomPoints[i], new Scalar(30, 250, 60), 3);
                            }
                        }//draw lines between stones if needed
                    }//don't focus on this right now

                    //create points to draw box around selected area
                    topLeft = new Point(Imgproc.boundingRect(c).x, Imgproc.boundingRect(c).y);
                    topRight = new Point(Imgproc.boundingRect(c).x, Imgproc.boundingRect(c).y + Imgproc.boundingRect(c).height);
                    bottomLeft = new Point(Imgproc.boundingRect(c).x + Imgproc.boundingRect(c).width, Imgproc.boundingRect(c).y);
                    bottomRight = new Point(Imgproc.boundingRect(c).x + Imgproc.boundingRect(c).width, Imgproc.boundingRect(c).y + Imgproc.boundingRect(c).height);
                }
            }
        }


        // This draws the contours that we found onto displayMat in a greenish color.
        //Imgproc.drawContours(displayMat, contoursYellowBig, -1, new Scalar(50, 230, 50), 2);
        //Draw contour with the biggest area in green
        Imgproc.drawContours(displayCrop, greatestContour, -1, new Scalar(30, 250, 60), 2);
        //Draw contour with the best height:width ratio in red
        //Imgproc.drawContours(displayMat, ratioContour, -1, new Scalar(250, 0, 0), 2);




        // Get the size (width, height) of the image.
        imageSize = displayMat.size();

        // Draw a horizontal line through the center as an example(the image is rotated)
        /*lineStart = new Point(imageSize.width/2, 0);
        lineEnd = new Point(imageSize.width/2, imageSize.height);
        Imgproc.line(displayMat, lineStart, lineEnd, colorRed, lineThickness, lineType);*/
        //Draw box around selected area in blue
        Imgproc.line(displayMat, topLeft, topRight, new Scalar(30, 30, 250), 3);
        Imgproc.line(displayMat, topLeft, bottomLeft, new Scalar(30, 30, 250), 3);
        Imgproc.line(displayMat, bottomLeft, bottomRight, new Scalar(30, 30, 250), 3);
        Imgproc.line(displayMat, bottomRight, topRight, new Scalar(30, 30, 250), 3);



        // If we detect the Skystone block state, we update currentDetectionState.
        //currentDetectionState.telemetry1 = "Nothing to see here.";


        // This gets displayMat back to the portrait mode that the rest of the pipeline is expecting.
        Core.transpose(displayMat,displayMat);
        //System.out.println(ratio);
        return displayMat;
    }

     @Override
    public void useDefaults() {
        // We don't need to do anything here, but we have to define this method.
     }
}
