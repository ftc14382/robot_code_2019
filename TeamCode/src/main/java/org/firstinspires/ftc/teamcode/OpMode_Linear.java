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

@TeleOp(name="Linear Opmode", group="Linear Opmode")
//@Disabled
public class OpMode_Linear extends LinearOpMode {
    public Mecanum chassis;
    public Function function;
    @Override
    public void runOpMode() {
        chassis = new Mecanum();
        chassis.init(hardwareMap, this);
        function = new Function();
        function.init(hardwareMap);
        telemetry.addData(":)", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double lifterPower;
        double h;
        double robotAngle;
        double maxSpeed;
        double turn;
        double speedChange;
        double v1;
        double v2;
        double v3;
        double v4;
        double functionSpeedChange;
        double grabberPower;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        chassis.runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            /*double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ; */

            //driving for mecanum wheels
            h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            v1 = -gamepad1.right_stick_y+gamepad1.right_stick_x;
            v2 = -gamepad1.right_stick_y - gamepad1.right_stick_x;
            //robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + Math.PI / 4;//Remember it is the angle moved + 45 degrees
            if(gamepad1.dpad_up) {
                h = 1;
                //robotAngle = Math.PI * 0.75;
                v1 = 1;
                v2 = 1;
            } else if(gamepad1.dpad_down) {
                h = 1;
                //robotAngle = Math.PI * 1.75;
                v1 = -1;
                v2 = -1;
            } else if(gamepad1.dpad_right) {
                h = 1;
                //robotAngle = Math.PI * 0.25;
                v1 = 1;
                v2 = -1;
            } else if(gamepad1.dpad_left) {
                h = 1;
                //robotAngle = Math.PI * 1.25;
                v1 = -1;
                v2 = 1;
            }
            if(gamepad1.b){
                turn = -0.8;
                h = 1;
            } else if(gamepad1.x) {
                turn = 0.8;
                h = 1;
            } else {
                turn = 0.0;
            }
            speedChange = 1-(gamepad1.right_trigger * 0.8);//Slow down the robot
            v4 = v1 + turn;
            v1 = v1 - turn;
            v3 = v2 + turn;
            v2 = v2 - turn;
            maxSpeed = Math.max(Math.abs(v1), Math.abs(v2));
            maxSpeed = Math.max(maxSpeed, Math.abs(v3));
            maxSpeed = Math.max(maxSpeed, Math.abs(v4));
            v1 = v1/maxSpeed*h*speedChange;
            v2 = v2/maxSpeed*h*speedChange;
            v3 = v3/maxSpeed*h*speedChange;
            v4 = v4/maxSpeed*h*speedChange;
            /*maxSpeed = Math.abs(Math.sin(robotAngle - Math.PI / 4) * Math.sqrt(2));
            //double turn = gamepad1.right_trigger - gamepad1.left_trigger;
            h *= speedChange;
            turn *= speedChange;
            v1 = h * Math.cos(robotAngle) * maxSpeed + turn;
            v2 = h * Math.sin(robotAngle) * maxSpeed + turn;
            v3 = h * Math.sin(robotAngle) * maxSpeed - turn;
            v4 = h * Math.cos(robotAngle) * maxSpeed - turn;*/

            if(gamepad1.left_stick_x != 0){//Normal, 2 wheeled drive
                v1=-gamepad1.left_stick_y+gamepad1.left_stick_x;
                v2=v1;
                v3=-gamepad1.left_stick_y-gamepad1.left_stick_x;
                v4=v3;
            }


            chassis.leftFront.setPower(v1);
            chassis.leftBack.setPower(v2);
            chassis.rightFront.setPower(v3);
            chassis.rightBack.setPower(v4);

            leftFrontPower = v1;
            leftBackPower = v2;
            rightFrontPower = v3;
            rightBackPower = v4;
            ///.clip?
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //leftDrive.setPower(leftPower);
            //rightDrive.setPower(rightPower);

            functionSpeedChange = 1-(gamepad2.right_trigger * 0.8);//Slow down the robot
            if(gamepad2.dpad_up) {
                lifterPower = 1.0;
            } else if(gamepad2.dpad_down) {
                lifterPower = -1.0;
            } else {
                lifterPower = 0.0;
            }
            function.lifter.setPower(functionSpeedChange*lifterPower);


            if (gamepad2.x) {
                grabberPower = -0.5;
            } else if (gamepad2.b) {
                grabberPower = 0.5;
            } else {
                grabberPower = 0.0;
            }
            function.grabber.setPower(functionSpeedChange*grabberPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + chassis.runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back (%.2f)",
                    leftBackPower, leftFrontPower, rightFrontPower, rightBackPower);
            telemetry.addData("Position", "left front (%d), left back (%d), right front (%d), right back (%d)", chassis.leftBack.getCurrentPosition(), chassis.leftFront.getCurrentPosition(), chassis.rightFront.getCurrentPosition(), chassis.rightBack.getCurrentPosition());
            telemetry.addData("Lifter", "power (%.2f)", lifterPower);
            telemetry.addData("Grabber","power (%.2f)", grabberPower);

            telemetry.update();
        }
    }
}
