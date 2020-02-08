package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="centertest", group="Linear OpMode")
public class CenterTest extends LinearOpMode{
    public enum Side{RED, BLUE}
    private static final Side startSide = Side.RED;
    public Mecanum chassis;
    public Function function;
    public CamSensor camSensor;
    private Position firstBlSetUp = new Position();
    private Position firstBl = new Position();
    private Position midPoint = new Position();
    private Position backup = new Position();
    private Position side = new Position();
    private Position line = new Position();
    private Position secondBlSetUp = new Position();
    private Position secondBl = new Position();
    private Position forwardBl1 = new Position();
    private Position twoInchMove = new Position();
    private double changeX;
    private double dis;
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
        camSensor.detector.detectorType = 0;//0=Skystone, 1=Red foundation, 2=Blue foundation
        String tag = "Detection";
        String tag2 = "Block";

        //Set up where the robot starts
        RobotInfo robotInfo = new RobotInfo();
        robotInfo.degrees = 0;
        robotInfo.x = 65*changeX;

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
            camSensor.detector.runTimes = 2;
            while(camSensor.detector.runTimes>1) {}
            if (camSensor.detector.currentDetectionState.detectedPosition > 240) {
                chassis.rightBack.setPower(-0.3);
                chassis.leftFront.setPower(-0.3);
                chassis.rightFront.setPower(0.3);
                chassis.leftBack.setPower(0.3);
            } else if (camSensor.detector.currentDetectionState.detectedPosition < 240) {
                chassis.rightBack.setPower(0.3);
                chassis.leftFront.setPower(0.3);
                chassis.rightFront.setPower(-0.3);
                chassis.leftBack.setPower(-0.3);
            } else break;
        }

    }

}


