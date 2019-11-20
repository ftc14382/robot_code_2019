//
// Class to hold information about a SkyStone detection.
//
package org.firstinspires.ftc.teamcode;


import org.opencv.core.Mat;

//
// detectedState should be:
//
//     0  - not detected
//     1  - Skystones in 1 and 4 spots
//     2  - Skystones in 2 and 5 spots
//     3  - skystones in 3 and6 spots
//
public class SkystoneDetectionState {
    public boolean detected       = false;
    public int     detectedState = 0;//1,2,3
    public int     detectedPosition = 0;//Pixel x position
    public String  telemetry1    = "";
    public String  telemetry2    = "";
    public Mat display;
}
