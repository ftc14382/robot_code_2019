package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

@Autonomous(name="AutoRedBuildWallFound", group="Linear OpMode")
public class AutoRedBuildWallFound extends LinearOpMode{
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
        forward.x = 48*changeX;
        forward.y = robotInfo.y;
        setUp.x = forward.x;
        setUp.y = 50;
        turn.x = setUp.x;
        turn.y = forward.y + 5*changeX;
        foundation.y = setUp.y;
        foundation.x = 29*changeX;
        turnFound1.y = 35;
        turnFound1.x = 60*changeX;
        turnFound2.x = turnFound1.x;
        turnFound2.y = turnFound1.y + 15;
        setUpLine.x = turnFound2.x;
        setUpLine.y = turnFound2.y - 10;
        line.x = 62*changeX;//was 70
        line.y = -2;



        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        chassis.driveTo(robotInfo, forward);
        chassis.turnTo(robotInfo, turn);
        chassis.quickDrive(robotInfo, setUp, 0.5, 1);
        chassis.quickDrive(robotInfo, foundation, 0.5, 1);
        function.foundMover.setPosition(0);
        sleep(300);
        /*for(Position p : foundationOutPositions) {
            chassis.quickDrive(robotInfo, p, 0.5);
        }*/
        function.grabber.setPower(-0.5);
        chassis.hardDrive(robotInfo, turnFound1, 0.7, 2, 1.9);
        chassis.turnAcurrate(robotInfo, -chassis.getIMUField()*1.9);//Turn farther because of load
        function.grabber.setPower(0);

        chassis.hardDrive(robotInfo, turnFound2, 1, 1.5, 1.9);
        robotInfo.y = 47.5;
        robotInfo.x = 48*changeX;
        /*chassis.turn(-90*changeX, 1, 1.3);
        robotInfo.degrees = chassis.getIMUField();*/
        function.foundMover.setPosition(1);
        sleep(200);
        chassis.quickDrive(robotInfo, setUpLine, 1, 2);
        chassis.turnAcurrate(robotInfo, 90);
        chassis.quickDrive(robotInfo, line, 1, 5);


        //For Camera
        /*detectionState = new SkystoneDetectionState();
        detectionState.detectedPosition = camSensor.detector.currentDetectionState.detectedPosition;
        detectionState.detectedState = camSensor.detector.currentDetectionState.detectedState;
        detectionState.telemetry1 = camSensor.detector.currentDetectionState.telemetry1;
        detectionState.telemetry2 = camSensor.detector.currentDetectionState.telemetry2;
        detectionState.display = camSensor.detector.currentDetectionState.display;
        telemetry.addData("Skystone State:", "%d", detectionState.detectedState);
        if (!detectionState.telemetry1.isEmpty()){
            telemetry.addData("Skystone: ",  detectionState.telemetry1 );
        }
        if (!detectionState.telemetry2.isEmpty()){
            telemetry.addData("Skystone: ", detectionState.telemetry2 );
        }
        telemetry.update();*/
        //Log detected info
        /*RobotLog.ii(tag, "Stone %d", detectionState.detectedState);
        RobotLog.ii(tag, "X Position: %2d", detectionState.detectedPosition);
        Date now = new Date();
        DateFormat dateFormat = new SimpleDateFormat("yyymmddhhmmss");
        String imgFileName = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES) + File.separator + dateFormat.format(now) + "-robocap.png";
        Imgcodecs.imwrite(imgFileName, detectionState.display);
        */
    }

}


