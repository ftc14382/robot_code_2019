package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.ArrayList;
import java.util.List;

public class CamSensor {
    public Dogeforia dogeforia;
    public WebcamName webcamName;
    public int cameraMonitorViewId;
    public SkystoneDetector detector;
    public boolean detectorEnabled = false;

    public void init(HardwareMap ahwMap) {
        webcamName = ahwMap.get(WebcamName.class, "Webcam 1"); //Retrieves the webcam from the hardware map
        detector   = new SkystoneDetector();
        //
        // A detector inherits from OpenCVPipeline in the DogeCV code. The init
        // function is just the init function on OpenCVPipeline.
        //
        // Detector parameters:
        //
        //   1. android.content.Context
        //   2. com.disnodeteam.dogecv.ViewDisplay
        //   3. com.disnodeteam.dogecv.CameraMode
        //   4. boolean (whether or not to find vumarks (i.e., run vuforia)
        //   5. org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
        detector.init(ahwMap.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, false, webcamName);
        detector.enable();
        detectorEnabled = true;
    }
}
