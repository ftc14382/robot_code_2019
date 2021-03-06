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
import java.util.Date;

@Autonomous(name="AutoRedLoadNeutral2Sky", group="Linear OpMode")
public class AutoRedLoadNeutral2Sky extends LinearOpMode{
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
        if(startSide == Side.RED) {
            changeX = 1;//1=red,-1=blue
            camSensor.detector.color = 0;//0=red, 1=blue
            robotInfo.degrees = 180;
            robotInfo.y = -40.5;//40.875
            backup.x = 34.5*changeX;
            secondBlSetUp.x = backup.x+0*changeX;//+3
        } else {
            changeX = -1;
            camSensor.detector.color = 1;
            robotInfo.degrees = 0;
            robotInfo.y = -42.5;//40.5
            backup.x = 40*changeX;//was 40
            secondBlSetUp.x = backup.x-3;//-8
        }
        robotInfo.x = 65*changeX;
        //Set up positions
        firstBlSetUp.x = robotInfo.x-2*changeX;
        firstBl.x = 26*changeX;//30, 28
        midPoint.x = firstBl.x + 12*changeX;
        side.x = backup.x;//always drifts to right -3
        side.y = 15;//was 10
        secondBl.x = firstBl.x;
        line.x = backup.x;//38
        line.y = 0.99;
        forwardBl1.x = 22.5*changeX;//24
        forwardBl1.y = -59;//was 57
        twoInchMove.x = robotInfo.x - 2*changeX;//2
        twoInchMove.y = robotInfo.y;

        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        //For Camera
        camSensor.detector.runTimes = 2;
        while(camSensor.detector.runTimes > 0) { }//Wait for the detection
        detectionState = new SkystoneDetectionState();
        detectionState.detectedPosition = camSensor.detector.currentDetectionState.detectedPosition;
        detectionState.detectedState = camSensor.detector.currentDetectionState.detectedState;
        detectionState.telemetry1 = camSensor.detector.currentDetectionState.telemetry1;
        detectionState.telemetry2 = camSensor.detector.currentDetectionState.telemetry2;
        //detectionState.display = camSensor.detector.currentDetectionState.display;
        telemetry.addData("Skystone State:", "%d", detectionState.detectedState);
        if (!detectionState.telemetry1.isEmpty()){
            telemetry.addData("Skystone: ",  detectionState.telemetry1 );
        }
        if (!detectionState.telemetry2.isEmpty()){
            telemetry.addData("Skystone: ", detectionState.telemetry2 );
        }
        telemetry.update();
        //Log detected info
        RobotLog.ii(tag, "Stone %d", detectionState.detectedState);
        RobotLog.ii(tag, "X Position: %2d", detectionState.detectedPosition);
        /*Date now = new Date();
        DateFormat dateFormat = new SimpleDateFormat("yyymmddhhmmss");
        String imgFileName = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES) + File.separator + dateFormat.format(now) + "-robocap.png";
        Imgcodecs.imwrite(imgFileName, detectionState.display);*/

        function.grabber.setPower(-0.008);//-0.008
        chassis.quickDrive(robotInfo, twoInchMove, 0.5, 0.5);
        function.grabber.setPower(0);

        if(detectionState.detectedState == 1) {
            firstBlSetUp.y =  -46;//Was -44, needed to fudge it
            RobotLog.ii(tag2, "Block 1");
            secondBlSetUp.y = -52;
        } else if(detectionState.detectedState == 2) {
            firstBlSetUp.y =  -35;
            RobotLog.ii(tag2, "Block 2");
            secondBlSetUp.y=-60;
        } else if(detectionState.detectedState == 3){
            firstBlSetUp.y =  -28;
            RobotLog.ii(tag2, "Block 3");
            secondBlSetUp.y=firstBlSetUp.y-24;
        }

        firstBl.y= firstBlSetUp.y;
        midPoint.y = firstBl.y;
        backup.y= firstBlSetUp.y;
        secondBl.y=secondBlSetUp.y;
        chassis.quickDrive(robotInfo,firstBlSetUp, 0.4, 2);
        chassis.driveTo(robotInfo, midPoint, 1,1.8);
        chassis.driveTo(robotInfo, firstBl, 0.75,1.5);

        //grab block
        function.grabber.setPower(-1);
        sleep(250);//720
        function.grabber.setPower(-0.5);
        //raise lifter slightly
        /*function.lifter.setPower(1);
        sleep(50);
        function.lifter.setPower(0);*/
        //Drive to other side
        backup.y = robotInfo.y;
        chassis.quickDrive(robotInfo, backup, 0.4, 2);
        chassis.driveTo(robotInfo, side);
        function.grabber.setPower(1);
        //function.lifter.setPower(-0.5);
        sleep(130);//205
        //function.lifter.setPower(0);
        function.grabber.setPower(0.5);

        chassis.quickDrive(robotInfo,secondBlSetUp);
        if(detectionState.detectedState == 1) {
            chassis.turnAcurrate(robotInfo, -180*changeX);
            secondBl.x = forwardBl1.x+4*changeX;
            chassis.quickDrive(robotInfo, secondBl);
            chassis.driveTo(robotInfo, forwardBl1, 1, 1);
        } else {
            function.grabber.setPower(-0.00003);
            chassis.driveTo(robotInfo,secondBl, 0.9,3);//Power was 0.76
        }
        function.grabber.setPower(-1);
        sleep(250);//720
        function.grabber.setPower(-0.5);
        //raise lifter slightly
        /*function.lifter.setPower(1);
        sleep(50);
        function.lifter.setPower(0);*/
        if(detectionState.detectedState == 1) {
            chassis.quickDrive(robotInfo, secondBl, 1, 2.01);
        }
        backup.y=secondBlSetUp.y;
        chassis.quickDrive(robotInfo, backup);
        chassis.driveTo(robotInfo, side, 1, 4);
        //function.grabber.setPower(0.8);
        chassis.quickDrive(robotInfo, line);
        function.grabber.setPower(0);

    }

}


