package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="turntest", group="Linear OpMode")
public class Test extends LinearOpMode{
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
        dis = chassis.leftFront.getCurrentPosition();
        chassis.rampDrive(1000, 1, 5);
        //chassis.rampTurn(180, 1, 5);
        //telemetry.addData("new angle", chassis.getIMUField());
        telemetry.addData("new distance", Math.abs(chassis.leftFront.getCurrentPosition() - dis)* chassis.COUNTS_PER_INCH_FORWARD);
        telemetry.update();
        //sleep(5000);
        //chassis.rampDrive(-24, 1, 5);
        telemetry.addData("new distance", Math.abs(chassis.leftFront.getCurrentPosition() - dis)* chassis.COUNTS_PER_INCH_FORWARD);
        //chassis.rampTurn(-90, 1, 5);
        //telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);
        /*
        chassis.turn(90, 0.75, 5);
        telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);
        chassis.turn(-90, 0.75, 5);
        telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);

         */
    }

}


