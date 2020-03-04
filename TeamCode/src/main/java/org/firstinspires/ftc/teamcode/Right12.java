package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Right12", group="Linear OpMode")
public class Right12 extends LinearOpMode{
    public enum Side{RED, BLUE}
    private static final Side startSide = Side.RED;
    public Mecanum chassis;
    public Function function;
    public CamSensor camSensor;
    private Position forward = new Position();
    private Position setUp = new Position();
    private Position turn = new Position();
    private Position foundation = new Position();
    List<Position> foundationOutPositions = new ArrayList<>();
    private Position turnFound1 = new Position();
    private Position turnFound2 = new Position();
    private Position line = new Position();
    private Position setUpLine = new Position();
    private double changeX = 1;//1=red,-1=blue.  Change this, robotinfo, and detector.color
    @Override
    public void runOpMode() {
        chassis = new Mecanum();
        chassis.init(hardwareMap, this, true);
        function = new Function();
        function.init(hardwareMap);
        //camSensor = new CamSensor();
        //camSensor.init(hardwareMap);
        //For Camera
        //SkystoneDetectionState detectionState;
        //camSensor.detector.detectorType = 0;//0=Skystone, 1=Red foundation, 2=Blue foundation
        //camSensor.detector.color = 0;//0=red, 1=blue.  This needs changed for different sides
        //String tag = "Detection";
        //String tag2 = "Block";

        //Set up where the robot starts
        RobotInfo robotInfo = new RobotInfo();
        if(startSide == Side.RED) {
            robotInfo.degrees = 180;
            changeX = 1;
        } else {
            robotInfo.degrees = 0;
            changeX = -1;
        }
        robotInfo.x = 65*changeX;
        robotInfo.y = 41.87;
        //Set up positions




        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        chassis.sideDrive(12, 1, 4);
    }

}


