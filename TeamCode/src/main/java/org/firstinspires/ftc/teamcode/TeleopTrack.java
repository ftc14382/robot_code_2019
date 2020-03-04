package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopTrack", group="Linear Opmode")
//@Disabled
public class TeleopTrack extends LinearOpMode {
    public Mecanum chassis;
    public Function function;
    @Override
    public void runOpMode() {
        chassis = new Mecanum();
        chassis.init(hardwareMap, this, true);
        function = new Function();
        function.init(hardwareMap);
        telemetry.addData(":)", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //for telemetry
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double servoPosition = 1;
        //normal drive
        double h;
        double robotAngle;
        double maxSpeed;
        double turn;
        double speedChange;
        double v1;
        double v2;
        double v3;
        double v4;
        //slow drive
        double v1s;
        double v2s;
        double v3s;
        double v4s;
        double hs;
        double functionSpeedChange;
        double grabberPower;
        double lifterPower;
        int bottomPos;
        int currentLifterPos;
        double maximum;

        int turnAngle;
        int lfturn;
        int lbturn;
        int rbturn;
        int rfturn;
        int COUNTS_PER_LEVEL;
        double startIMUAngle;
        double offset;
        double offsetDegrees;
        ElapsedTime myTimer = new ElapsedTime();
        double timeLast, timeNow, deltaT;
        double hLast = 0.0;
        final double maxAccel = 1;
        double accel;
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        chassis.runtime.reset();
        startIMUAngle = 90;//0
        bottomPos = function.lifter.getCurrentPosition() - 50;
        currentLifterPos = function.lifter.getCurrentPosition();

        COUNTS_PER_LEVEL = 0;
        // run until the end of the match (driver presses STOP)


        timeLast = myTimer.time();
        while (opModeIsActive()) {
            //driving for mecanum wheels
            h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

            timeNow = myTimer.time();
            deltaT = timeNow - timeLast;
            if (deltaT < 0.0001) {
                continue;
            }
            accel = Math.abs((h - hLast)/deltaT);
            if (accel > maxAccel) {
                if (h > 0) {
                    h = maxAccel * deltaT + hLast;
                } else {
                    h = 0;
                }
            }
            hLast = h;
            timeLast = timeNow;
            v1 = -gamepad1.right_stick_y+gamepad1.right_stick_x;
            v2 = -gamepad1.right_stick_y - gamepad1.right_stick_x;

            if (gamepad1.y) startIMUAngle = 180 + chassis.getIMUAngle(); //Red
            if (gamepad1.b) startIMUAngle = 90 + chassis.getIMUAngle();
            if (gamepad1.a) startIMUAngle = 0 + chassis.getIMUAngle(); //Blue
            if (gamepad1.x) startIMUAngle = 270 + chassis.getIMUAngle();

            //if (gamepad1.a) startIMUAngle = chassis.getIMUAngle();
            //trig
            offsetDegrees = startIMUAngle-chassis.getIMUAngle();
            offset = Math.toRadians(offsetDegrees);
            turn = gamepad1.right_trigger - gamepad1.left_trigger;
            //h = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
            robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) + offset; //+offset
            v1 = h * (Math.sin(robotAngle) + Math.cos(robotAngle));
            v2 = h * (Math.sin(robotAngle) - Math.cos(robotAngle));
            v3 = v2;
            v4 = v1;
            if(gamepad1.dpad_up) {
               v1 = 0.5;
               v2 = 0.5;
               v3 = 0.5;
               v4 = 0.5;
            } else if(gamepad1.dpad_down) {
                v1 = -0.5;
                v2 = -0.5;
                v3 = -0.5;
                v4 = -0.5;
            } else if(gamepad1.dpad_right) {
                v1 = 0.5;
                v4 = 0.5;
                v2 = -0.5;
                v3 = -0.5;
            } else if(gamepad1.dpad_left) {
                v1 = -0.5;
                v4 = -0.5;
                v2 = 0.5;
                v3 = 0.5;
            }

            //trig
            h = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) + offset;
            v1s = .5 * h * (Math.sin(robotAngle) + Math.cos(robotAngle));
            v2s = .5 * h * (Math.sin(robotAngle) - Math.cos(robotAngle));
            v3s = v2s;
            v4s = v1s;

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                v1 = v1s;
                v2 = v2s;
                v3 = v3s;
                v4 = v4s;
            }
            //set power

            if (!(chassis.leftFront.isBusy() || chassis.rightFront.isBusy())) {
                v1 += turn;
                v2 += turn;
                v3 -= turn;
                v4 -= turn;
                maximum = 1;
                if (Math.abs(v1) > 1 || Math.abs(v2) > 1 || Math.abs(v3) > 1 || Math.abs(v4) > 1) {
                    maximum = Math.max(Math.abs(v1), Math.abs(v2));
                    maximum = Math.max(maximum, Math.abs(v3));
                    maximum = Math.max(maximum, Math.abs(v4));
                }
                chassis.leftFront.setPower(v1 / maximum);
                chassis.leftBack.setPower(v2 / maximum);
                chassis.rightFront.setPower(v3 / maximum);
                chassis.rightBack.setPower(v4 / maximum);
            }

            //for telemetry
            leftFrontPower = v1;
            leftBackPower = v2;
            rightFrontPower = v3;
            rightBackPower = v4;

            functionSpeedChange = 1-(gamepad2.right_trigger * 0.8);//Slow down the robot

            if (gamepad2.x) {
                grabberPower = -0.5 - gamepad2.left_trigger*0.5;
            } else if (gamepad2.b) {
                grabberPower = 0.5 + gamepad2.left_trigger*0.5;
            } else {
                grabberPower = 0.0;
            }
            function.grabber.setPower(functionSpeedChange*grabberPower);


            if(gamepad2.left_stick_y != 0) {
                bottomPos = function.lifter.getCurrentPosition() - 50;
            }
            //normal lifter control
            functionSpeedChange = 1-(gamepad2.right_trigger * 0.8);//Slow down the robot
            if(gamepad2.dpad_up) {
                function.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lifterPower = 1.0;
                currentLifterPos= function.lifter.getCurrentPosition();
                function.lifter.setPower(functionSpeedChange*lifterPower);
            } else if(gamepad2.dpad_down && function.lifter.getCurrentPosition() > bottomPos) {
                function.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lifterPower = -0.8;
                currentLifterPos= function.lifter.getCurrentPosition();
                function.lifter.setPower(functionSpeedChange*lifterPower);
            } else {
                function.lifter.setTargetPosition(currentLifterPos);
                function.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                function.lifter.setPower(0.2);
            }


            if (gamepad2.left_bumper) {
                function.lifter.setTargetPosition(function.lifter.getCurrentPosition() + COUNTS_PER_LEVEL);
            }
            if (gamepad2.right_bumper) {
                function.lifter.setTargetPosition(function.lifter.getCurrentPosition() - COUNTS_PER_LEVEL);
            }

            if(gamepad2.left_bumper) {
                servoPosition = 1;
                //servoPosition = 0.5;
            } else if(gamepad2.right_bumper) {
                servoPosition = 0;
                //servoPosition = -0.5;
            } else{
                //servoPosition = 0;
            }
            function.foundMover.setPosition(servoPosition);
            //function.foundMover2.setPower(servoPosition);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + chassis.runtime.toString());
            telemetry.addData("Lifter", "Position: %d", function.lifter.getCurrentPosition());
            telemetry.addData("Turn", "Turn Value: " + turn);
            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back (%.2f)",
                    leftBackPower, leftFrontPower, rightFrontPower, rightBackPower);
            telemetry.addData("Position", "left front (%d), left back (%d), right front (%d), right back (%d)", chassis.leftBack.getCurrentPosition(), chassis.leftFront.getCurrentPosition(), chassis.rightFront.getCurrentPosition(), chassis.rightBack.getCurrentPosition());
            //telemetry.addData("Lifter", "power (%.2f)", lifterPower);
            //telemetry.addData("Grabber","power (%.2f)", grabberPower);

            telemetry.update();
        }
    }
}
