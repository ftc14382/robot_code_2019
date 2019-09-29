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

    private static final String VUFORIA_KEY = "ASiIF9r/////AAABmbB85zU3k0g3qzF1DLbC7GUnvGVHWKDgtHLp6I/mzHMkcRm8A0oZl2woG1jqog81fIG7hAfVTp50Fj3sgLTQCqJ/sy9mZ/SQzMh2E3EBTIqS4ndxzRR0KGqW62bmVqQN69a7cuamH1QC4y3yiTaEDha8JoQF7kS3K32S6bziY2MYoBO8PCegD6dsnhtAH4VnAwIeiM/dCvhDXh1FuLFfLZmoExZGKasu20D3hqlvVRFoa7jUIIdzEEbuCM70asfMyzHk1ZdqgpBAqFOtxoyVgF0/ackncBT+hYFqfBbPkFGwiLiFED/8OBiMWRLVm4raAYo9NIgXqDFJhghNXqL8OMPwyuYYJuhZfqeg0z39M3fr";
    
    //Useful constants.
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)

    public CamSensor camSensor;

    public void runOpMode() {
        SkystoneDetectionState detectionState;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        camSensor = new CamSensor();
        camSensor.VUFORIA_KEY = VUFORIA_KEY;
        camSensor.init(hardwareMap);

        waitForStart();

        // Run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                camsensor.saveSnapshot();
//            }

            detectionState = camSensor.detector.currentDetectionState;

            telemetry.addData("Gamepad1:", "%s", gamepad1.toString());
            telemetry.addData("Skystone State:", "%d", detectionState.detectedState);
        }
    }
}
