/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import javax.sql.RowSetEvent;


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

@TeleOp(name="NewTeleOp", group="Linear Opmode")
//@Disabled
public class NewTeleOp extends LinearOpMode {
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
        while (opModeIsActive()) {
            //driving for mecanum wheels
            h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            v1 = -gamepad1.right_stick_y + gamepad1.right_stick_x;
            v2 = -gamepad1.right_stick_y - gamepad1.right_stick_x;
            //robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + Math.PI / 4;//Remember it is the angle moved + 45 degrees

            //non trig
            /*turn = (gamepad1.right_trigger - gamepad1.left_trigger);
            if (gamepad1.left_bumper) {
                turn = -.3;
            } else if (gamepad1.right_bumper) {
                turn = .3;
            }
            //speedChange = 1-(gamepad1.right_trigger * 0.8);//Slow down the robot
            v1 = v1;
            v2 = v2;
            v3 = v2;
            v4 = v1;
            maxSpeed = Math.max(Math.abs(v1), Math.abs(v2));
            maxSpeed = Math.max(maxSpeed, Math.abs(v3));
            maxSpeed = Math.max(maxSpeed, Math.abs(v4));
            if (maxSpeed != 0) {
                v1 = (v1 / maxSpeed) * h;
                v2 = (v2 / maxSpeed) * h;
                v3 = (v3 / maxSpeed) * h;
                v4 = (v4 / maxSpeed) * h;
            } else {
                v1 = 0;
                v2 = 0;
                v3 = 0;
                v4 = 0;
            }
            */

            if (gamepad1.y) startIMUAngle = 90; //Red
            if (gamepad1.b) startIMUAngle = 180;
            if (gamepad1.a) startIMUAngle = 0; //Blue
            if (gamepad1.x) startIMUAngle = 270;

            if (gamepad1.a) startIMUAngle = chassis.getIMUAngle();
            //trig
            offsetDegrees = startIMUAngle - chassis.getIMUAngle();
            offset = Math.toRadians(offsetDegrees);
            turn = gamepad1.right_trigger - gamepad1.left_trigger;
            h = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
            robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) + offset; //+offset
            v1 = h * (Math.sin(robotAngle) + Math.cos(robotAngle));
            v2 = h * (Math.sin(robotAngle) - Math.cos(robotAngle));
            v3 = v2;
            v4 = v1;
            if (gamepad1.dpad_up) {
                v1 = 0.5;
                v2 = 0.5;
                v3 = 0.5;
                v4 = 0.5;
            } else if (gamepad1.dpad_down) {
                v1 = -0.5;
                v2 = -0.5;
                v3 = -0.5;
                v4 = -0.5;
            } else if (gamepad1.dpad_right) {
                v1 = 0.5;
                v4 = 0.5;
                v2 = -0.5;
                v3 = -0.5;
            } else if (gamepad1.dpad_left) {
                v1 = -0.5;
                v4 = -0.5;
                v2 = 0.5;
                v3 = 0.5;
            }

            //left stick is half speed
            //non trig
            /*
            hs = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            v1s = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            v2s = -gamepad1.left_stick_y - gamepad1.left_stick_x;

            v1s = v1s;
            v2s = v2s;
            v3s = v2s;
            v4s = v1s;

            v1s = (v1s / 2) * hs;
            v2s = (v2s / 2) * hs;
            v3s = (v3s / 2) * hs;
            v4s = (v4s / 2) * hs;
            */
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
            /*
            if (gamepad1.left_bumper) {
                turnAngle = -90;

                turnAngle *= chassis.COUNTS_PER_DEGREE;
                lfturn = chassis.leftFront.getCurrentPosition() - turnAngle;
                lbturn = chassis.leftBack.getCurrentPosition() - turnAngle;
                rfturn = chassis.rightFront.getCurrentPosition() + turnAngle;
                rbturn = chassis.rightBack.getCurrentPosition() + turnAngle;

                chassis.leftFront.setTargetPosition(lfturn);
                chassis.leftBack.setTargetPosition(lbturn);
                chassis.rightFront.setTargetPosition(rfturn);
                chassis.rightBack.setTargetPosition(rbturn);

                chassis.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chassis.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chassis.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chassis.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                chassis.leftFront.setPower(1);
                chassis.leftBack.setPower(1);
                chassis.rightFront.setPower(1);
                chassis.rightBack.setPower(1);


            } else if (gamepad1.right_bumper) {
                turnAngle = 90 - (int)chassis.getIMUAngle();
                /*
                turnAngle *= chassis.COUNTS_PER_DEGREE;
                lfturn = chassis.leftFront.getCurrentPosition() + turnAngle;
                lbturn = chassis.leftBack.getCurrentPosition() + turnAngle;
                rfturn = chassis.rightFront.getCurrentPosition() - turnAngle;
                rbturn = chassis.rightBack.getCurrentPosition() - turnAngle;

                chassis.leftFront.setTargetPosition(lfturn);
                chassis.leftBack.setTargetPosition(lbturn);
                chassis.rightFront.setTargetPosition(rfturn);
                chassis.rightBack.setTargetPosition(rbturn);

                chassis.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chassis.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chassis.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chassis.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                chassis.leftFront.setPower(1);
                chassis.leftBack.setPower(1);
                chassis.rightFront.setPower(1);
                chassis.rightBack.setPower(1);

                if (chassis.getIMUAngle() < turnAngle) {
                    chassis.leftFront.setPower(1);
                    chassis.leftBack.setPower(1);
                    chassis.rightFront.setPower(-1);
                    chassis.rightBack.setPower(-1);
                } else {
                    chassis.leftFront.setPower(0);
                    chassis.leftBack.setPower(0);
                    chassis.rightFront.setPower(0);
                    chassis.rightBack.setPower(0);
                }

            }
            */

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
            /*
            COUNTS_PER_LEVEL = 1;
            lifterPos = function.lifter.getCurrentPosition();
            if (gamepad2.dpad_up) {
                function.lifter.setTargetPosition(lifterPos + COUNTS_PER_LEVEL);
            } else if(gamepad2.dpad_down) {
                function.lifter.setTargetPosition(lifterPos - COUNTS_PER_LEVEL);
            } else if (!function.lifter.isBusy()) {
                function.lifter.setPower(0);
            }

            function.lifter.setPower(1);
            */


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
