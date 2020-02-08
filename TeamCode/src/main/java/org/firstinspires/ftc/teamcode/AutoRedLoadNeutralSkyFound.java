package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="AutoRedLoadNeutralSkyFound", group="Linear OpMode")
public class AutoRedLoadNeutralSkyFound extends LinearOpMode{
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

    private Position moveOut = new Position();
    private Position putOnFound = new Position();

    private Position setUp = new Position();
    private Position turn = new Position();
    private Position foundation = new Position();
    private Position turnFound1 = new Position();
    private Position turnFound2 = new Position();

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
            robotInfo.y = -40.875;//41.25?
            backup.x = 40*changeX;
            secondBlSetUp.x = backup.x - 5;
        } else {
            changeX = -1;
            camSensor.detector.color = 1;
            robotInfo.degrees = 0;
            robotInfo.y = -41.75;
            backup.x = 37*changeX;
            secondBlSetUp.x = backup.x - 8;
        }
        robotInfo.x = 65*changeX;
        //Set up positions
        firstBlSetUp.x = robotInfo.x-2*changeX;
        firstBl.x = 30*changeX;
        midPoint.x = firstBl.x + 12*changeX;
        side.x = backup.x-3;//always drifts to right
        side.y = 5;//was 10
        secondBl.x = firstBl.x;
        line.x = 38*changeX;
        line.y = 0.99;
        forwardBl1.x = 23*changeX;
        forwardBl1.y = -59;//was 57
        twoInchMove.x = robotInfo.x - 2*changeX;
        twoInchMove.y = robotInfo.y;

        /*sideFoundTransition.x = 48*changeX;
        sideFoundTransition.y = */
        putOnFound.x = 31;
        putOnFound.y = 50;

        //Foundation
        setUp.x = 48*changeX;
        setUp.y = putOnFound.y;
        turn.x = setUp.x;
        turn.y = 42 + 5*changeX;
        foundation.y = setUp.y;
        foundation.x = 29*changeX;
        turnFound1.y = 35;
        turnFound1.x = 60*changeX;
        turnFound2.x = turnFound1.x;
        turnFound2.y = turnFound1.y + 15;
        moveOut.y = 35;

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

        function.grabber.setPower(-0.008);
        chassis.quickDrive(robotInfo, twoInchMove);
        function.grabber.setPower(0);

        if(detectionState.detectedState == 1) {
            firstBlSetUp.y =  -46;//Was -44, needed to fudge it
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
        chassis.quickDrive(robotInfo,firstBlSetUp, 0.4, 1.2);
        chassis.driveTo(robotInfo, midPoint, 1);
        chassis.driveTo(robotInfo, firstBl, 0.75);

        //grab block
        function.grabber.setPower(-1);
        sleep(720);
        function.grabber.setPower(-0.5);
        backup.y = robotInfo.y;
        chassis.quickDrive(robotInfo, backup, 0.4, 1);
        //chassis.driveTo(robotInfo, side);
        chassis.driveTo(robotInfo, setUp);
        chassis.driveTo(robotInfo, putOnFound);

        function.grabber.setPower(1);
        sleep(205);
        function.grabber.setPower(0.5);


        //Move foundation
        chassis.quickDrive(robotInfo, setUp);
        chassis.turnTo(robotInfo, turn);
        chassis.quickDrive(robotInfo, setUp, 0.5, 1);
        chassis.quickDrive(robotInfo, foundation, 0.5, 1);
        function.foundMover.setPosition(0);
        sleep(300);
        chassis.hardDrive(robotInfo, turnFound1, 0.7, 2, 1.9);
        chassis.turnAcurrate(robotInfo, -chassis.getIMUField()*1.9);//Turn farther because foundation drags
        chassis.hardDrive(robotInfo, turnFound2, 1, 1.5, 1.9);
        robotInfo.y = 47.5;
        robotInfo.x = 48*changeX;
        function.foundMover.setPosition(1);
        sleep(200);

        moveOut.x = robotInfo.x;
        chassis.quickDrive(robotInfo, moveOut, 0.3, 3);

        chassis.quickDrive(robotInfo,secondBlSetUp);
        if(detectionState.detectedState == 1) {
            chassis.turnAcurrate(robotInfo, 175);
            secondBl.x = forwardBl1.x;
            chassis.quickDrive(robotInfo, secondBl);
            chassis.driveTo(robotInfo, forwardBl1);
        } else {
            chassis.driveTo(robotInfo,secondBl, 0.76);
        }
        function.grabber.setPower(-1);
        sleep(720);
        function.grabber.setPower(-0.5);
        //raise lifter slightly
        /*function.lifter.setPower(1);
        sleep(50);
        function.lifter.setPower(0);*/
        if(detectionState.detectedState == 1) {
            chassis.quickDrive(robotInfo, secondBl, 1, 1);
        }
        backup.y=secondBlSetUp.y;
        chassis.quickDrive(robotInfo, backup);
        chassis.driveTo(robotInfo, side);
        function.grabber.setPower(0.8);
        chassis.quickDrive(robotInfo, line);
        function.grabber.setPower(0);

    }

}


