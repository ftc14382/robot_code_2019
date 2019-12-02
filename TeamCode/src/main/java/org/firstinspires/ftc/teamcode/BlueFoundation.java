package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Foundation Move(BLUE)", group="Linear OpMode")
public class BlueFoundation extends LinearOpMode{
    public Mecanum chassis;
    public Function function;
    public CamSensor camSensor;
    private Position forward = new Position();
    private Position setUp = new Position();
    private Position turn = new Position();
    private Position foundation = new Position();
    List<Position> foundationOutPositions = new ArrayList<>();
    private Position templateP = new Position();
    private Position line = new Position();
    private double changeX = -1;//1=red,-1=blue.  Change this, robotinfo, and detector.color
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
        robotInfo.x = 65*changeX;
        robotInfo.y = 41.87;
        robotInfo.degrees = 180;//This needs changed for different sides
        //Set up positions
        forward.x = 48*changeX;
        forward.y = robotInfo.y;
        setUp.x = forward.x;
        setUp.y = 50.5;
        turn.x = setUp.x;
        turn.y = setUp.y + 5*changeX;
        foundation.y = setUp.y;
        foundation.x = 29*changeX;
        templateP.y = foundation.y;
        for(int i=1; i<9; i ++) {
            templateP.x = foundation.x + 4.5*i*changeX;
            foundationOutPositions.add(templateP);
        }
        line.x = 65*changeX;
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
        chassis.quickDrive(robotInfo, setUp);
        chassis.quickDrive(robotInfo, foundation);
        function.foundMover.setPosition(0);
        for(Position p : foundationOutPositions) {
            chassis.quickDrive(robotInfo, p);
        }
        chassis.turn(-90*changeX, 1, 1.3);
        robotInfo.degrees = chassis.getIMUField();
        function.foundMover.setPosition(0.7);
        chassis.quickDrive(robotInfo, line);


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


