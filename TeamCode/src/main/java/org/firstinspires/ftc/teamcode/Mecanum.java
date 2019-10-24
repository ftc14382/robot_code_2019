package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanum {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    public LinearOpMode robot;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * Math.PI);


    static final double COUNTS_PER_INCH = 1;
    static final double COUNTS_PER_DEGREE = 1;

    public void init(HardwareMap ahwMap, LinearOpMode Arobot) {
        robot = Arobot;
        leftFront = ahwMap.get(DcMotor.class, "left_front");
        leftBack = ahwMap.get(DcMotor.class, "left_back");
        rightFront = ahwMap.get(DcMotor.class, "right_front");
        rightBack = ahwMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void simpleDrive(double distance, double dir) {
        leftFront.setPower(dir);
        leftBack.setPower(dir);
        rightFront.setPower(dir);
        rightBack.setPower(dir);
        if (robot.opModeIsActive()) {
            robot.telemetry.addData("Path", "Running forwards %2d inches", distance);
            robot.telemetry.update();
            while (robot.opModeIsActive() &&
                    leftFront.getCurrentPosition() < distance * COUNTS_PER_INCH) {
                robot.telemetry.addData("Path", "Running. . .");
                robot.telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turn(double degrees, int dir) {
        leftFront.setPower(dir);
        leftBack.setPower(dir);
        rightFront.setPower(-dir);
        rightBack.setPower(-dir);

        if (robot.opModeIsActive()) {
            robot.telemetry.addData("Path", "Turning %7d degrees", degrees);
            robot.telemetry.update();


            while (robot.opModeIsActive() &&
                    leftFront.getCurrentPosition() < COUNTS_PER_DEGREE * degrees){
                robot.telemetry.addData("Path", "Turning. . .");
                robot.telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void sideDrive(double distance, double dir) {
        leftFront.setPower(dir);
        leftBack.setPower(-dir);
        rightFront.setPower(dir);
        rightBack.setPower(-dir);
        if (robot.opModeIsActive()) {
            robot.telemetry.addData("Path", "Moving %2d inches sideways", distance);
            robot.telemetry.update();

            while (robot.opModeIsActive() &&
                    leftFront.getCurrentPosition() < COUNTS_PER_INCH * distance){
                robot.telemetry.addData("Path", "Crab Walking. . .");
                robot.telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void encoderDrive(double upInches, double rightInches) {
        double distance = Math.hypot(upInches, rightInches);
        double robotAngle = Math.atan2(upInches, rightInches) + Math.PI / 4;
        //double maxSpeed = Math.abs(Math.sin(robotAngle) * Math.sqrt(2));
        double power = distance * Math.cos(robotAngle);
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        if (robot.opModeIsActive()) {
            robot.telemetry.addData("Path", "Moving towards %2d, %2d", upInches, rightInches);
            robot.telemetry.update();
            while (robot.opModeIsActive() &&
                    leftFront.getCurrentPosition() < distance * COUNTS_PER_INCH) {
                robot.telemetry.addData("Path", "Running. . .");
                robot.telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
