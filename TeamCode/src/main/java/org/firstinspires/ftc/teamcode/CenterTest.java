package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Mat;

@Autonomous(name="centertest", group="Linear OpMode")
public class CenterTest extends LinearOpMode{
    public enum Side{RED, BLUE}
    private static final Side startSide = Side.RED;
    public Mecanum chassis;
    public Function function;
    public CamSensor camSensor;
    double power;

    @Override
    public void runOpMode() {
        chassis = new Mecanum();
        chassis.init(hardwareMap, this, true);
        function = new Function();
        function.init(hardwareMap);
        camSensor = new CamSensor();
        camSensor.init(hardwareMap);
        //For Camera
        SkystoneDetectionState detectionState;
        camSensor.detector.detectorType = 3;//0=Skystone, 1=Red foundation, 2=Blue foundation, 3=yellow stone


        //Set up where the robot starts
        RobotInfo robotInfo = new RobotInfo();
        robotInfo.degrees = 0;


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();
        camSensor.detector.runTimes = 2;
        while(camSensor.detector.runTimes>1){}
        while (opModeIsActive() && camSensor.detector.currentDetectionState.detected) {
            camSensor.detector.runTimes = 1;
            /*while(camSensor.detector.runTimes>1) {}
            if (camSensor.detector.currentDetectionState.detectedPosition < 240) {
                chassis.rightBack.setPower(-0.4);
                chassis.leftFront.setPower(-0.4);
                chassis.rightFront.setPower(0.4);
                chassis.leftBack.setPower(0.4);
            } else if (camSensor.detector.currentDetectionState.detectedPosition > 240) {
                chassis.rightBack.setPower(0.4);
                chassis.leftFront.setPower(0.4);
                chassis.rightFront.setPower(-0.4);
                chassis.leftBack.setPower(-0.4);
            } else break;*/

            if(Math.abs(camSensor.detector.currentDetectionState.detectedPosition - 240) < 7) {
                break;
            }
            power = (camSensor.detector.currentDetectionState.detectedPosition-240)*0.0018;
            if(Math.abs(power)>1) {
                power /= Math.abs(power);
            }
            chassis.rightBack.setPower(power);
            chassis.leftFront.setPower(power);
            chassis.rightFront.setPower(-power);
            chassis.leftBack.setPower(-power);
        }

    }

}


