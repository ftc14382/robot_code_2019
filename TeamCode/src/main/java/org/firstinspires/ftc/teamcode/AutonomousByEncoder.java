package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous by Encoder", group="Linear OpMode")
public class AutonomousByEncoder extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            //(WHEEL_DIAMETER_INCHES * Math.PI);

    static final double COUNTS_PER_INCH = 1;
    static final double COUNTS_PER_DEGREE = 1;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.


    @Override
    public void runOpMode() {
        telemetry.addData(":)", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Ready", "Starting at %7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        simpleDrive(12, 1);
        turn(90, 1);
        sideDrive(12, 1);
        //encoderDrive(36.0, 36.0, 30.0);
    }
        public void simpleDrive(double distance, double dir) {
            leftFront.setPower(dir);
            leftBack.setPower(dir);
            rightFront.setPower(dir);
            rightBack.setPower(dir);
            if (opModeIsActive()) {
                telemetry.addData("Path", "Running forwards %2d inches", distance);
                telemetry.update();
                while (opModeIsActive() &&
                        leftFront.getCurrentPosition() < distance * COUNTS_PER_INCH) {
                    telemetry.addData("Path", "Running. . .");
                    telemetry.update();
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

            if (opModeIsActive()) {
                telemetry.addData("Path", "Turning %7d degrees", degrees);
                telemetry.update();


                while (opModeIsActive() &&
                        leftFront.getCurrentPosition() < COUNTS_PER_DEGREE * degrees){
                    telemetry.addData("Path", "Turning. . .");
                    telemetry.update();
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

            if (opModeIsActive()) {
                telemetry.addData("Path", "Moving %2d inches sideways", distance);
                telemetry.update();


                while (opModeIsActive() &&
                        leftFront.getCurrentPosition() < COUNTS_PER_INCH * distance){
                    telemetry.addData("Path", "Crab Walking. . .");
                    telemetry.update();
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
            double maxSpeed = Math.abs(Math.sin(robotAngle) * Math.sqrt(2));
            double power = distance * Math.cos(robotAngle) * maxSpeed;
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            if (opModeIsActive()) {
                telemetry.addData("Path", "Moving towards %2d, %2d", upInches, rightInches);
                telemetry.update();
                while (opModeIsActive() &&
                        leftFront.getCurrentPosition() < distance * COUNTS_PER_INCH) {
                    telemetry.addData("Path", "Running. . .");
                    telemetry.update();
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


