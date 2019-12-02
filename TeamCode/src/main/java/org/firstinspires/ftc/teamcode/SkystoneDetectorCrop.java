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
    private Mat cropMask    = new Mat();
    public SkystoneDetectionState currentDetectionState;
    public int detectorType = 0;//0=skyStone, 1=redFoundation, 2=blueFoundation
    public int color = 0;//0=red, 1=blue


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


        // Copy the input mat to our working mats, then release it for memory.
        // Note that the image that we get is portrait mode. The transpose
        // below makes it more intuitive to program: Image Y axis corresponds to
        // camera Y axis.
        // The transpose on displayMat is undone right before we return it.
        Core.transpose(input,displayMat);
        displayMat.copyTo(workingMat);
        input.release();

        //Define upper and lower range of color
        Scalar lowerMask = new Scalar(0,0,0);
        Scalar upperMask = new Scalar(1,1,1);
        //Define what area we are cropping
        int cropX = 0;
        int cropY = 0;
        int cropWidth = 0;
        int cropHeight = 0;

        if(detectorType == 0) {
            //Define upper and lower range of color
            lowerMask = new Scalar(10, 40, 40);//was(20, 50, 50)  I am trying to change it so it sees better
            upperMask = new Scalar(60, 255, 255);//was(40, 255, 255)
            //Define what area we are cropping
            cropX = 0;
            cropY = 270;//Was 160
            cropWidth = 480;
            cropHeight = 100;
        } //Skystone
        else if(detectorType == 1) {
            //Define upper and lower range of color
            lowerMask = new Scalar(10, 50, 40);//The color red is 0 hue so we either have to create two masks or find all of the non-red and invert it
            upperMask = new Scalar(170, 255, 255);//The range that these masks create is all the color that is not red
            //Define what area we are cropping
            cropX = 0;
            cropY = 0;
            cropWidth = 480;
            cropHeight = 640;
        } //Red foundation(hue value is from 0-180)
        else if(detectorType == 2) {
            //Define upper and lower range of color
            lowerMask = new Scalar(110, 50, 19);
            upperMask = new Scalar(130, 255, 250);
            //Define what area we are cropping
            cropX = 0;
            cropY = 0;
            cropWidth = 640;
            cropHeight = 480;
        } //Blue foundation




        // Create an HSV copy of workingMat and detect the yellow/blue/red "Hue".
        Imgproc.cvtColor(workingMat, workingMatHsv, Imgproc.COLOR_RGB2HSV);
        // The output of inRange is a "grayscale" image. A grayscale image
        // only has one number per pixel (usually called "Y"). Compare to
        // BGR which has three numbers per pixel (Blue, Green, and Red).
        // In fact, the output of inRange is an image in which all pixels are
        // either one or zero. This is sometimes called a "binary image" or
        // simply "a mask".
        Core.inRange(workingMatHsv, lowerMask, upperMask, maskYellow);

        //Invert and crop
        Rect rectCrop = new Rect(cropX, cropY, cropWidth, cropHeight);//look at https://stackoverflow.com/questions/35666255/get-a-sub-image-using-opencv-java
        if(detectorType == 0 || detectorType == 1) {
            Mat reverseMask = new Mat();
            Core.bitwise_not(maskYellow, reverseMask);//Invert mask
            cropMask = new Mat(reverseMask, rectCrop);//Crop mask

            // Now we take our grayscale maskYellow image and create an RGB image.
            Imgproc.cvtColor(reverseMask, displayMat, Imgproc.COLOR_GRAY2RGB);
        }//For skystone and red foundation
        else {
            cropMask = new Mat(maskYellow, rectCrop);//Crop mask

            // Now we take our grayscale maskYellow image and create an RGB image.
            Imgproc.cvtColor(maskYellow, displayMat, Imgproc.COLOR_GRAY2RGB);
        }//For blue foundations



        //Crop image
        Mat displayCrop = new Mat(displayMat, rectCrop);

        // This finds the contours in the yellowMask image.
        List<MatOfPoint> contoursColor = new ArrayList<>();
        List<MatOfPoint> greatestContour = new ArrayList<>();

        //Create points for rectangle that shows where we are cropping
        Point topLeft = new Point(cropX, cropY);
        Point topRight = new Point(cropX, cropY+cropHeight);
        Point bottomRight = new Point(cropX+cropWidth, cropY+cropHeight);
        Point bottomLeft = new Point(cropX+cropWidth, cropY);

        Point circleCenter = new Point(0,0);//Initalize point for center of the circle

        //Initialize variables
        double areaCrop;
        double maxAreaCrop = 0.0;
        int detectedX=0;
        Imgproc.findContours(cropMask, contoursColor, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);//Find contours
        for (MatOfPoint c : contoursColor) {
            areaCrop = Imgproc.contourArea(c);//Detect biggest yellow area
            if (areaCrop > maxAreaCrop) {
                maxAreaCrop = areaCrop;
                greatestContour.clear();
                greatestContour.add(c);//Make sure we are only selecting one countour
                detectedX = (int)(Imgproc.boundingRect(c).x + Imgproc.boundingRect(c).width/2);//Middle of selected area
                circleCenter = new Point(detectedX, (int)(Imgproc.boundingRect(c).y + Imgproc.boundingRect(c).height/2));//Create point for the center of the selected image
            }
        }
        Imgproc.circle(displayCrop, circleCenter, 3, new Scalar(250, 10,10), -1);//Draw circle in middle of slected area
        //Draw contour with the biggest area in green
        Imgproc.drawContours(displayCrop, greatestContour, -1, new Scalar(30, 250, 60), 2);
        // Get the size (width, height) of the image.
        imageSize = displayMat.size();

        //Draw box around selected area in blue
        Imgproc.line(displayMat, topLeft, topRight, new Scalar(30, 30, 250), 3);
        Imgproc.line(displayMat, topLeft, bottomLeft, new Scalar(30, 30, 250), 3);
        Imgproc.line(displayMat, bottomLeft, bottomRight, new Scalar(30, 30, 250), 3);
        Imgproc.line(displayMat, bottomRight, topRight, new Scalar(30, 30, 250), 3);


        if(color == 1 && detectorType == 0) {
            if(detectedX > 210 && detectedX<300) {//270,180
                currentDetectionState.detectedState = 1;
            }
            else if(detectedX > 90 && detectedX < 180) {//390,300
                currentDetectionState.detectedState = 2;
            }
            else if((detectedX < 325 && detectedX > 425) || (detectedX < 80 && detectedX > 30)) {//155,55 and 450,400
                currentDetectionState.detectedState = 3;
            }
        }//blue side
        else if(color == 0) {
            if (detectedX < 270 && detectedX > 180 && detectorType == 0) {
                currentDetectionState.detectedState = 1;
            } else if (detectedX < 390 && detectedX > 300 && detectorType == 0) {
                currentDetectionState.detectedState = 2;
            } else if (((detectedX < 155 && detectedX > 55) || (detectedX < 450 && detectedX > 400)) && detectorType == 0 ) {
                currentDetectionState.detectedState = 3;
            }
        }//Red side


        // If we detect something, we update currentDetectionState.
        if(detectorType == 0 && (maxAreaCrop<50000 && maxAreaCrop>9000)) {
            currentDetectionState.telemetry1 = "Skystone found!";
            currentDetectionState.telemetry2 = detectedX + "," + maxAreaCrop;
            currentDetectionState.detected = true;
            currentDetectionState.detectedPosition = detectedX;
        }
        else if((detectorType == 1 || detectorType == 2) && maxAreaCrop > 700) {
            currentDetectionState.telemetry1 = "Foundation found!";
            currentDetectionState.telemetry2 = "" + detectedX;
            currentDetectionState.detected = true;
            currentDetectionState.detectedPosition = detectedX;
        }
        else{
            currentDetectionState.telemetry1 = "Nothing to see here.";
            currentDetectionState.telemetry2 = "M.A.C." + maxAreaCrop;
            currentDetectionState.detected = false;
            currentDetectionState.detectedPosition = 0;
        }
        // This gets displayMat back to the portrait mode that the rest of the pipeline is expecting.
        Core.transpose(displayMat,displayMat);
        //System.out.println(ratio);
        currentDetectionState.display = displayMat;
        return displayMat;
    }

     @Override
    public void useDefaults() {
        // We don't need to do anything here, but we have to define this method.
     }
}
