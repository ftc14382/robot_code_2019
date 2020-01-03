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
    private double changeX = 1;//1=red,-1=blue.  Change this, robotinfo, and detector.color!!!
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
        camSensor.detector.color = 0;//0=red, 1=blue.  This needs changed for different sides!!!!!
        String tag = "Detection";
        String tag2 = "Block";

        //Set up where the robot starts
        RobotInfo robotInfo = new RobotInfo();
        robotInfo.x = 65*changeX;
        robotInfo.y = -40.875;
        robotInfo.degrees = 180;//This needs changed for different sides!!!!
        //Set up positions
        firstBlSetUp.x = robotInfo.x-2*changeX;
        firstBl.x = 30*changeX;
        midPoint.x = firstBl.x + 8*changeX;
        backup.x = 40*changeX;//Was 44
        side.x = backup.x;//changed
        side.y = 15;
        secondBlSetUp.x = backup.x;
        secondBl.x = firstBl.x;
        line.x = 40*changeX;//changed
        line.y = 0.99;



        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        //For Camera
        detectionState = new SkystoneDetectionState();
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
        telemetry.update();
        //Log detected info
        RobotLog.ii(tag, "Stone %d", detectionState.detectedState);
        RobotLog.ii(tag, "X Position: %2d", detectionState.detectedPosition);
        Date now = new Date();
        DateFormat dateFormat = new SimpleDateFormat("yyymmddhhmmss");
        String imgFileName = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES) + File.separator + dateFormat.format(now) + "-robocap.png";
        Imgcodecs.imwrite(imgFileName, detectionState.display);

        chassis.simpleDrive(2,1);
        robotInfo.x -= 2*changeX;

        if(detectionState.detectedState == 1) {
            firstBlSetUp.y =  -44;
            RobotLog.ii(tag2, "Block 1");
            secondBlSetUp.y = -52;
        } else if(detectionState.detectedState == 2) {
            firstBlSetUp.y =  -36;
            RobotLog.ii(tag2, "Block 2");
            secondBlSetUp.y=firstBlSetUp.y-24;
        } else if(detectionState.detectedState == 3){
            firstBlSetUp.y =  -28;
            RobotLog.ii(tag2, "Block 3");
            secondBlSetUp.y=firstBlSetUp.y-24;
        }

        firstBl.y= firstBlSetUp.y;
        midPoint.y = firstBl.y;
        backup.y= firstBlSetUp.y;
        secondBl.y=secondBlSetUp.y;
        chassis.quickDrive(robotInfo,firstBlSetUp, 0.4);
        chassis.driveTo(robotInfo, midPoint, 0.9);
        chassis.driveTo(robotInfo, firstBl, 0.6);

        //grab block
        function.grabber.setPower(-0.8);
        sleep(900);
        function.grabber.setPower(-0.5);
        //raise lifter slightly
        function.lifter.setPower(1);
        sleep(50);
        function.lifter.setPower(0);
        //Drive to other side
        chassis.quickDrive(robotInfo, backup, 0.4);
        chassis.driveTo(robotInfo, side);
        function.grabber.setPower(0.8);
        function.lifter.setPower(-0.4);
        sleep(258);
        function.lifter.setPower(0);
        function.grabber.setPower(0.5);

        chassis.quickDrive(robotInfo,secondBlSetUp);
        if(detectionState.detectedState == 1) {
            chassis.turnAcurrate(robotInfo, 180);
            secondBl.x = 23*changeX;
            chassis.quickDrive(robotInfo, secondBl);
        } else {
            chassis.driveTo(robotInfo,secondBl, 0.6);
        }
        function.grabber.setPower(-0.8);
        sleep(900);
        function.grabber.setPower(-0.5);
        //raise lifter slightly
        function.lifter.setPower(1);
        sleep(50);
        function.lifter.setPower(0);
        backup.y=secondBlSetUp.y;
        chassis.quickDrive(robotInfo, backup);
        chassis.driveTo(robotInfo, side);
        function.grabber.setPower(0.8);
        chassis.quickDrive(robotInfo, line);
        function.grabber.setPower(0);

    }

}

