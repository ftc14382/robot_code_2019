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
    private Position start = new Position();
    private Position end = new Position();
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
        robotInfo.degrees = 180;
        robotInfo.x = 0*changeX;
        robotInfo.y = 0;

        start.x = robotInfo.x;
        start.y = robotInfo.y;
        end.x = start.x - 24;
        end.y = start.y;

        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();
        dis = chassis.leftFront.getCurrentPosition();
        //chassis.sideDrive(500, 1, 3);

        //telemetry.addData("new distance", Math.abs(chassis.leftFront.getCurrentPosition() - dis)* chassis.COUNTS_PER_INCH_FORWARD);
        //chassis.rampTurn(360, 1, 5);
        chassis.sideDrive(500, 1, 5);
        telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);

        /*chassis.turn(-360, 1, 5);
        telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);*/
        /*
        chassis.turn(-90, 0.75, 5);
        telemetry.addData("new angle", chassis.getIMUField());
        telemetry.update();
        sleep(5000);*/

    }

}


