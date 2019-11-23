package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class Mecanum {
    public IMU iMU;
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


    static final double COUNTS_PER_INCH_FORWARD = 30.36;
    static final double COUNTS_PER_INCH_SIDE = 32.29;
    static final double COUNTS_PER_DEGREE = 4.79;

    public void init(HardwareMap ahwMap, LinearOpMode Arobot, boolean useIMU) {
        robot = Arobot;
        if(useIMU) {
            iMU = new IMU();
            iMU.init(ahwMap, Arobot);
        }
        leftFront = ahwMap.get(DcMotor.class, "left_front");
        leftBack = ahwMap.get(DcMotor.class, "left_back");
        rightFront = ahwMap.get(DcMotor.class, "right_front");
        rightBack = ahwMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        /*leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void simpleDrive(double distance, double dir) {
        int lFTarget;
        int lBTarget;
        int rFTarget;
        int rBTarget;
        if(robot.opModeIsActive()) {
            lFTarget = leftFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_FORWARD);
            lBTarget = leftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_FORWARD);
            rFTarget = rightFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_FORWARD);
            rBTarget = rightBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_FORWARD);

            leftFront.setTargetPosition(lFTarget);
            leftBack.setTargetPosition(lBTarget);
            rightFront.setTargetPosition(rFTarget);
            rightBack.setTargetPosition(rBTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(dir);
            leftBack.setPower(dir);
            rightFront.setPower(dir);
            rightBack.setPower(dir);

        }
        robot.telemetry.addData("Path", "Running forwards %.2f inches", distance);
        robot.telemetry.update();
        while (robot.opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
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

    public void turn(double degrees, double dir) {
        int lFTarget;
        int lBTarget;
        int rFTarget;
        int rBTarget;
        if(robot.opModeIsActive()) {
            lFTarget = leftFront.getCurrentPosition() - (int)(degrees * COUNTS_PER_DEGREE);
            lBTarget = leftBack.getCurrentPosition() - (int)(degrees * COUNTS_PER_DEGREE);
            rFTarget = rightFront.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            rBTarget = rightBack.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);

            leftFront.setTargetPosition(lFTarget);
            leftBack.setTargetPosition(lBTarget);
            rightFront.setTargetPosition(rFTarget);
            rightBack.setTargetPosition(rBTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(dir);
            leftBack.setPower(dir);
            rightFront.setPower(dir);
            rightBack.setPower(dir);

        }

        robot.telemetry.addData("Path", "Turning %.2f degrees", degrees);
        robot.telemetry.update();


        while (robot.opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
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

    public void sideDrive(double distance, double dir) {//Positive is to the left
        int lFTarget;
        int lBTarget;
        int rFTarget;
        int rBTarget;
        if(robot.opModeIsActive()) {
            lFTarget = leftFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_SIDE);
            lBTarget = leftBack.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH_SIDE);
            rFTarget = rightFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH_SIDE);
            rBTarget = rightBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH_SIDE);

            leftFront.setTargetPosition(lFTarget);
            leftBack.setTargetPosition(lBTarget);
            rightFront.setTargetPosition(rFTarget);
            rightBack.setTargetPosition(rBTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(dir);
            leftBack.setPower(dir);
            rightFront.setPower(dir);
            rightBack.setPower(dir);

        }
            robot.telemetry.addData("Path", "Moving %.2f inches sideways", distance);
            robot.telemetry.update();

            while (robot.opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
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

    public void driveTo(RobotInfo r, Position p/*, boolean brake*/) {
        String tag = "Drive To";
        RobotLog.ii(tag, "Start Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;

        double startIMUangle = getIMUAngle();//for loging
        double IMUTurned;

        //RobotLog.ii(Tag, "driveTo: initial(x,y,degrees): %.2f, %.2f, %.2f", r.x, r.y, r.degrees);
        //RobotLog.ii(Tag, "driveTo: target(x,y,degrees): %.2f, %.2f", p.x, p.y);

        if (turn > 180) {
            turn -= 360;
        }
        if (turn < -180) {
            turn += 360;
        }
        //RobotLog.ii(Tag, "DriveTo: comanded angle: %.2f", turn);
        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }*/


        turn(turn, 0.5);
        simpleDrive(distance, 0.7);

        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
        IMUTurned = getIMUField();

        r.x = p.x;
        r.y = p.y;
        if(Math.abs(IMUTurned - Math.toDegrees(theta)) < 9) {
            r.degrees = getIMUField();
        } else {
            r.degrees = Math.toDegrees(theta);
        }
        RobotLog.ii(tag, "End Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);

        /*telemetry.addData("RobotX:", r.x);
        telemetry.addData("RobotY", r.y);
        telemetry.addData("Robot Heading", r.degrees);*/
        //RobotLog.ii(Tag, "DriveTo: turned IMU angle: %.2f", getIMUAngle() - startIMUangle);
    }

    public void quickDrive(RobotInfo r, Position p/*, boolean brake*/) {
        String tag = "Quick Drive";
        RobotLog.ii(tag, "Start Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;
        double turnBack;
        double turnSide;

        double startIMUangle = getIMUAngle();//for loging
        double IMUTurned;

        //RobotLog.ii(Tag, "driveTo: initial(x,y,degrees): %.2f, %.2f, %.2f", r.x, r.y, r.degrees);
        //RobotLog.ii(Tag, "driveTo: target(x,y,degrees): %.2f, %.2f", p.x, p.y);

        //Make sure the robot isn't turing more than 180
        if (turn > 180) {
            turn -= 360;
        }
        if (turn < -180) {
            turn += 360;
        }

        //Calculate values
        turnBack = turn - 180;
        turnSide = turn - 90;
        //Go backwards
        if (turnBack > 180) {
            turnBack -= 360;
        } else if (turnBack < -180) {
            turnBack += 360;
        }


        //Go sideways
        if (turnSide > 90) {
            turnSide -= 180;
        } else if (turnSide < -90) {
            turnSide += 180;
        }
        if(Math.abs(turnSide) < 45){
            turn = turnSide;
        } else if(Math.abs(turnBack) < 45){
            turn = turnBack;
            distance *= -1;
        }

        //RobotLog.ii(Tag, "DriveTo: comanded angle: %.2f", turn);
        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }*/


        turn(turn, 1);//was 0.5
        if(Math.abs(turnSide) < 45) {
            sideDrive(distance, 1);
        } else {
            simpleDrive(distance, 1);//was 0.7
        }

        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
        IMUTurned = getIMUField();

        r.x = p.x;
        r.y = p.y;
        r.degrees = getIMUField();
        RobotLog.ii(tag, "End Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);

        /*telemetry.addData("RobotX:", r.x);
        telemetry.addData("RobotY", r.y);
        telemetry.addData("Robot Heading", r.degrees);*/
        //RobotLog.ii(Tag, "DriveTo: turned IMU angle: %.2f", getIMUAngle() - startIMUangle);
    }

    public void turnTo(RobotInfo r, Position p) {
        String tag = "Turn to";
        RobotLog.ii(tag, "Start Pos: %.2f", r.degrees);
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;
        double startIMUangle = getIMUAngle();//for loging
        double IMUTurned;

        //RobotLog.ii(Tag, "driveTo: initial(x,y,degrees): %.2f, %.2f, %.2f", r.x, r.y, r.degrees);
        //RobotLog.ii(Tag, "driveTo: target(x,y,degrees): %.2f, %.2f", p.x, p.y);

        if (turn > 180) {
            turn -= 360;
        }
        if (turn < -180) {
            turn += 360;
        }
        //RobotLog.ii(Tag, "DriveTo: comanded angle: %.2f", turn);
        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }*/


        turn(turn, 0.9);

        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
        IMUTurned = getIMUField();


        if(Math.abs(IMUTurned - Math.toDegrees(theta)) < 9) {
            r.degrees = getIMUField();
        } else {
            r.degrees = Math.toDegrees(theta);
        }
        RobotLog.ii(tag, "End Pos: %.2f", r.degrees);

        /*telemetry.addData("RobotX:", r.x);
        telemetry.addData("RobotY", r.y);
        telemetry.addData("Robot Heading", r.degrees);*/
        //RobotLog.ii(Tag, "DriveTo: turned IMU angle: %.2f", getIMUAngle() - startIMUangle);
    }

    public double getIMUAngle() {
        return iMU.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getIMUField() {
        double actual = getIMUAngle() + iMU.startIMUOffset;
        //RobotLog.ii(Tag, "getIMUField: returned angle: %.2f", actual);
        return(actual);
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
                    leftFront.getCurrentPosition() < distance * COUNTS_PER_INCH_FORWARD) {
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
