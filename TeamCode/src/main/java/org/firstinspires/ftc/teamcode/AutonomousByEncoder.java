package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import org.opencv.imgcodecs.Imgcodecs;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.text.DateFormat;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.io.File;

@Autonomous(name="Autonomous by Encoder(RED)", group="Linear OpMode")
public class AutonomousByEncoder extends LinearOpMode{
    public Mecanum chassis;
    public Function function;
    public CamSensor camSensor;
    private Position forward = new Position();
    private Position turntoPosition = new Position();
    private Position bl1SetUp = new Position();
    private Position bl2SetUp = new Position();
    private Position bl3SetUp = new Position();
    private Position bl1 = new Position();
    private Position bl2 = new Position();
    private Position bl3 = new Position();
    private Position backup = new Position();
    private Position side = new Position();
    private Position leave = new Position();
    private Position line = new Position();
    private double changeX = 1;//1=red,-1=blue.  Change this, robotinfo, and detector.color
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
        camSensor.detector.color = 0;//0=red, 1=blue.  This needs changed for different sides
        String tag = "Detection";
        String tag2 = "Block";

        //Set up where the robot starts
        RobotInfo robotInfo = new RobotInfo();
        robotInfo.x = 65*changeX;
        robotInfo.y = -40.1;
        robotInfo.degrees = 180;//This needs changed for different sides
        //Set up positions
        forward.x = 55*changeX;
        forward.y = -40.1;
        turntoPosition.x = forward.x;
        turntoPosition.y = forward.y - 5*changeX;
        bl1SetUp.x = forward.x;
        bl1SetUp.y = -44;
        bl2SetUp.x = forward.x;
        bl2SetUp.y = -36;
        bl3SetUp.x = forward.x;
        bl3SetUp.y = -28;
        bl1.x = 32*changeX;
        bl1.y = bl1SetUp.y;
        bl2.x = bl1.x;
        bl2.y = bl2SetUp.y;
        bl3.x = bl1.x;
        bl3.y = bl3SetUp.y;
        backup.x = 61*changeX;
        side.x = 61*changeX;//changed
        side.y = 15;
        leave.x = 60.5*changeX;//60
        leave.y = 19;
        line.x = 61*changeX;//changed
        line.y = 0.99;


        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        chassis.driveTo(robotInfo, forward);
        chassis.turnTo(robotInfo, turntoPosition);
        sleep(1000);

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



        if(detectionState.detectedState == 1) {
            chassis.quickDrive(robotInfo,bl1SetUp);
            chassis.driveTo(robotInfo,bl1);
            backup.y = bl1.y;
            RobotLog.ii(tag2, "Block 1");
        } else if(detectionState.detectedState == 2) {
            chassis.quickDrive(robotInfo,bl2SetUp);
            chassis.driveTo(robotInfo,bl2);
            backup.y = bl2.y;
            RobotLog.ii(tag2, "Block 2");
        } else if(detectionState.detectedState == 3){
            chassis.quickDrive(robotInfo,bl3SetUp);
            chassis.driveTo(robotInfo,bl3);
            backup.y = bl3.y;
            RobotLog.ii(tag2, "Block 3");
        }

        //grab block
        function.grabber.setPower(-0.8);
        sleep(900);
        function.grabber.setPower(-0.5);
        //raise lifter slightly
        function.lifter.setPower(1);
        sleep(90);
        function.lifter.setPower(0);
        //Drive to other side
        chassis.quickDrive(robotInfo, backup);
        chassis.quickDrive(robotInfo, side);
        //release skystone
        function.grabber.setPower(0.8);
        sleep(900);
        function.grabber.setPower(0);
        chassis.turnTo(robotInfo, leave);
        //park on line
        chassis.quickDrive(robotInfo, line);


        //chassis.simpleDrive(3, 1);
        //chassis.sideDrive(40, 1);

        //chassis.turn(90, 1);
        //chassis.sideDrive(12, 1);
        //encoderDrive(36.0, 36.0, 30.0);
    }

}


