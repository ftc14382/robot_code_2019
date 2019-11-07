package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="Autonomous by Encoder", group="Linear OpMode")
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

        //Set up where the robot starts
        RobotInfo robotInfo = new RobotInfo();
        robotInfo.x = 65;
        robotInfo.y = -40.1;
        robotInfo.degrees = 180;
        //Set up positions
        forward.x = 55;
        forward.y = -40.1;
        turntoPosition.x = forward.x;
        turntoPosition.y = forward.y - 5;
        bl1SetUp.x = forward.x;
        bl1SetUp.y = -42.5;
        bl2SetUp.x = forward.x;
        bl2SetUp.y = -34.75;
        bl3SetUp.x = forward.x;
        bl3SetUp.y = -26.75;
        bl1.x = 32;
        bl1.y = bl1SetUp.y;
        bl2.x = bl1.x;
        bl2.y = bl2SetUp.y;
        bl3.x = bl1.x;
        bl3.y = bl3SetUp.y;
        backup.x = 61;
        side.x = 61;
        side.y = 12;

        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        telemetry.addData(":)", "Initialized");
        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        chassis.driveTo(robotInfo, forward);
        chassis.turnTo(robotInfo, turntoPosition);
        sleep(500);

        //For Camera
        detectionState = camSensor.detector.currentDetectionState;
        telemetry.addData("Skystone State:", "%d", detectionState.detectedState);
        if (!detectionState.telemetry1.isEmpty()){
            telemetry.addData("Skystone: ",  detectionState.telemetry1 );
        }
        if (!detectionState.telemetry2.isEmpty()){
            telemetry.addData("Skystone: ", detectionState.telemetry2 );
        }
        telemetry.update();


        if(detectionState.detectedState == 1) {
            chassis.quickDrive(robotInfo,bl1SetUp);
            chassis.driveTo(robotInfo,bl1);
            backup.y = bl1.y;
        } else if(detectionState.detectedState == 2) {
            chassis.quickDrive(robotInfo,bl2SetUp);
            chassis.driveTo(robotInfo,bl2);chassis.driveTo(robotInfo,bl1);
            backup.y = bl2.y;
        } else if(detectionState.detectedState == 3){
            chassis.quickDrive(robotInfo,bl3SetUp);
            chassis.driveTo(robotInfo,bl3);chassis.driveTo(robotInfo,bl1);
            backup.y = bl3.y;
        }

        function.grabber.setPower(0.8);
        sleep(900);
        function.grabber.setPower(-0.5);

        chassis.quickDrive(robotInfo, backup);
        chassis.quickDrive(robotInfo, side);


        //chassis.simpleDrive(3, 1);
        //chassis.sideDrive(40, 1);

        //chassis.turn(90, 1);
        //chassis.sideDrive(12, 1);
        //encoderDrive(36.0, 36.0, 30.0);
    }

}


