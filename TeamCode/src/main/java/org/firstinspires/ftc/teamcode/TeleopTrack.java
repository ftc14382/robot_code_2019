package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;


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
        RobotInfo robotInfo = new RobotInfo();
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
        //normal drive
        double h;
        double robotAngle;
        double turn;
        double maximum;

        double startIMUAngle;
        double offset;
        double offsetDegrees;
        ElapsedTime myTimer = new ElapsedTime();
        double timeLast, timeNow, deltaT;
        double hLast = 0.0;
        final double maxAccel = 1;
        double accel;
        final double angleFactor = Math.sqrt(2)/2;

        double previousIMU;
        double currentIMU;
        double differenceIMU;
        int previousLF;
        int previousLB;
        int previousRF;
        int previousRB;
        int currentLF;
        int currentLB;
        int currentRF;
        int currentRB;
        int differenceLF;
        int differenceLB;
        int differenceRF;
        int differenceRB;
        double totalMovement;
        double lFPercent;
        double lBPercent;
        double rFPercent;
        double rBPercent;
        double lFX;
        double lBX;
        double rFX;
        double rBX;
        double lFY;
        double lBY;
        double rFY;
        double rBY;

        double directionAngle;
        double distanceMoved;
        double forceX;
        double forceY;
        double distanceX;
        double distanceY;

        Position startPosition = new Position();
        startPosition.x = 0;
        startPosition.y = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        chassis.runtime.reset();
        startIMUAngle = 90;//0
        robotInfo.degrees = 90;
        robotInfo.x = 0;
        robotInfo.y = 0;
        chassis.iMU.startIMUOffset = robotInfo.degrees - chassis.getIMUAngle();

        previousLF = chassis.leftFront.getCurrentPosition();
        previousLB = chassis.leftBack.getCurrentPosition();
        previousRF = chassis.rightFront.getCurrentPosition();
        previousRB = chassis.rightBack.getCurrentPosition();
        previousIMU = chassis.getIMUAngle();


        // run until the end of the match (driver presses STOP)


        timeLast = myTimer.time();
        while (opModeIsActive()) {
            //driving for mecanum wheels
            h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            currentIMU = chassis.getIMUAngle();
            timeNow = myTimer.time();
            deltaT = timeNow - timeLast;
            if (deltaT < 0.0001) {
                continue;
            }
            robotInfo.degrees = chassis.getIMUField();
            //Tracking code
            //Get wheel position
            currentLF = chassis.leftFront.getCurrentPosition();
            currentLB = chassis.leftBack.getCurrentPosition();
            currentRF = chassis.rightFront.getCurrentPosition();
            currentRB = chassis.rightBack.getCurrentPosition();
            //Get the difference in the encoders
            differenceLF = previousLF - currentLF;
            differenceLB = previousLB - currentLB;
            differenceRF = previousRF - currentRF;
            differenceRB = previousRB - currentRB;
            //Take out any turning
            differenceIMU = previousIMU - currentIMU;//Positive means we have turned clockwise
            if(differenceIMU > 180) {
                differenceIMU -= 360;
            } else if(differenceIMU < -180) {
                differenceIMU += 360;
            }
            differenceLF -= differenceIMU*chassis.COUNTS_PER_DEGREE;
            differenceLB -= differenceIMU*chassis.COUNTS_PER_DEGREE;
            differenceRF += differenceIMU*chassis.COUNTS_PER_DEGREE;
            differenceRB += differenceIMU*chassis.COUNTS_PER_DEGREE;
            //Get the x and y values for each wheel
            lFX = differenceLF*angleFactor;
            lFY = differenceLF*angleFactor;
            lBX = differenceLB*angleFactor*-1;
            lBY = differenceLB*angleFactor;
            rFX = differenceRF*angleFactor*-1;
            rFY = differenceRF*angleFactor;
            rBX = differenceRB*angleFactor;
            rBY = differenceRB*angleFactor;
            //Get how much each wheel moved out of total movement
            totalMovement = Math.abs(differenceLF)+Math.abs(differenceLB)+Math.abs(differenceRF)+Math.abs(differenceRB);
            lFPercent = Math.abs(differenceLF)/totalMovement;
            lBPercent = Math.abs(differenceLB)/totalMovement;
            rFPercent = Math.abs(differenceRF)/totalMovement;
            rBPercent = Math.abs(differenceRB)/totalMovement;
            //Calculate Values for the whole robot
            forceY = lFY+lBY+rFY+rBY;
            forceX = lFX+lBX+rFX+rBX;
            directionAngle = Math.toDegrees(Math.atan2(forceY, forceX)) + robotInfo.degrees;
            distanceY = lFY*lFPercent+lBY*lBPercent+rFY*rFPercent+rBY*rBPercent;
            distanceX = lFX*lFPercent+lBX*lBPercent+rFX*rFPercent+rBX*rBPercent;
            distanceMoved = Math.sqrt(distanceY*distanceY+distanceX*distanceX);
            robotInfo.x = Math.cos(Math.toRadians(directionAngle))*distanceMoved+robotInfo.x;
            robotInfo.y = Math.sin(Math.toRadians(directionAngle))*distanceMoved+robotInfo.y;


            //Press down on the left stick to go to where the robot started to see if it the tracking works
            if(gamepad1.left_stick_button){
                chassis.driveTo(robotInfo, startPosition, 0.8, 10);
            }


            //Aceleration
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

            //Adjust field-oriented driving angle
            if (gamepad1.y) startIMUAngle = 180 + currentIMU; //Red
            if (gamepad1.b) startIMUAngle = 90 + currentIMU;
            if (gamepad1.a) startIMUAngle = 0 + currentIMU; //Blue
            if (gamepad1.x) startIMUAngle = 270 + currentIMU;

            //Calculate right stick(fast)
            offsetDegrees = startIMUAngle-currentIMU;
            offset = Math.toRadians(offsetDegrees);
            turn = gamepad1.right_trigger - gamepad1.left_trigger;
            robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) + offset; //+offset
            leftFrontPower = h * (Math.sin(robotAngle) + Math.cos(robotAngle));
            leftBackPower = h * (Math.sin(robotAngle) - Math.cos(robotAngle));
            rightFrontPower = leftBackPower;
            rightBackPower = leftFrontPower;



            //Keep it below one and set power
            if (!(chassis.leftFront.isBusy() || chassis.rightFront.isBusy())) {
                leftFrontPower += turn;
                leftBackPower += turn;
                rightFrontPower -= turn;
                rightBackPower -= turn;
                maximum = 1;
                if (Math.abs(leftFrontPower) > 1 || Math.abs(leftBackPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightBackPower) > 1) {
                    maximum = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
                    maximum = Math.max(maximum, Math.abs(rightFrontPower));
                    maximum = Math.max(maximum, Math.abs(rightBackPower));
                }
                chassis.leftFront.setPower(leftFrontPower / maximum);
                chassis.leftBack.setPower(leftBackPower / maximum);
                chassis.rightFront.setPower(rightFrontPower / maximum);
                chassis.rightBack.setPower(rightBackPower / maximum);
            }







            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + chassis.runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back (%.2f)",
                    leftBackPower, leftFrontPower, rightFrontPower, rightBackPower);
            telemetry.addData("Position", "left front (%d), left back (%d), right front (%d), right back (%d)", chassis.leftBack.getCurrentPosition(), chassis.leftFront.getCurrentPosition(), chassis.rightFront.getCurrentPosition(), chassis.rightBack.getCurrentPosition());
            telemetry.update();

            previousIMU = currentIMU;
            previousLF = currentLF;
            previousLB = currentLB;
            previousRF = currentRF;
            previousRB = currentRB;
        }
    }
}
