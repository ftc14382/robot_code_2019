//
// Class to hold information about a SkyStone detection.
//
package org.firstinspires.ftc.teamcode;


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
    public int     detectedState = 0;
    public int     detectedPosition = 0;
    public String  telemetry1    = "";
    public String  telemetry2    = "";
}
