// This line identifies the package that we are a part of.
// We can use classes in the same package without import statements.
package org.firstinspires.ftc.teamcode;

// Opmode classes from FTC.
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Classes for doing navigation
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

// Basic java classes
import java.util.ArrayList;
import java.util.List;

// This picks out our menu location in the Driver Station GUI.
@TeleOp(name="StoneDetect", group ="Examples")
// Uncomment the following line to disable this opmode. That is, to prevent
// it from appearing in the Driver Station GUI. 
//@Disabled

// Java has one class per file. The class the same name (minus ".java")
// as the file.
public class StoneDetectionExample extends LinearOpMode {

    //Useful constants.
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)

    public CamSensor camSensor;

    public void runOpMode() {
        SkystoneDetectionState detectionState;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        camSensor = new CamSensor();
        camSensor.init(hardwareMap);

        camSensor.detector.detectorType = 0;//0=Skyston, 1=Red foundation, 2=Blue foundation

        waitForStart();

        // Run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                camsensor.saveSnapshot();
//            }
            camSensor.detector.runTimes = 1;
            detectionState = camSensor.detector.currentDetectionState;

            telemetry.addData("Gamepad1:", "%s", gamepad1.toString());
            telemetry.addData("Skystone State:", "%d", detectionState.detectedState);

            if (!detectionState.telemetry1.isEmpty()){
                telemetry.addData("Skystone: ",  detectionState.telemetry1 );
            }
            if (!detectionState.telemetry2.isEmpty()){
                telemetry.addData("Skystone: ", detectionState.telemetry2 );
            }


            // Send telemetry.
            telemetry.update();
        }
    }
}
