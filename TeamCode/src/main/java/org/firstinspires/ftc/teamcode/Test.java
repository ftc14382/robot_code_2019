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
        //Set up positions
        firstBlSetUp.x = robotInfo.x-2*changeX;
        firstBl.x = 30*changeX;
        midPoint.x = firstBl.x + 12*changeX;
        side.x = backup.x-3;//always drifts to right
        side.y = 10;//was 15
        secondBl.x = firstBl.x;
        line.x = 38*changeX;
        line.y = 0.99;
        forwardBl1.x = 23*changeX;
        forwardBl1.y = -59;//was 57
        twoInchMove.x = robotInfo.x - 2*changeX;
        twoInchMove.y = robotInfo.y;

        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        chassis.turn(250, 1, 5);
        telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);
    }

}


