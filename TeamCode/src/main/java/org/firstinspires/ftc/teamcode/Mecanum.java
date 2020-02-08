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
    public Planner planner;


    public LinearOpMode robot;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * Math.PI);


    static final double COUNTS_PER_INCH_FORWARD = 30.36/2;
    static final double COUNTS_PER_INCH_SIDE = 32.29/2;
    static final double COUNTS_PER_DEGREE = 2.8225;//4.68/2

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        /*leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            //robot.telemetry.addData("Path", "Running. . .");
            //robot.telemetry.update();
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

    public void turn(double degrees, double dir, double timeoutS) {
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
        runtime.reset();


        while (robot.opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy() && (runtime.seconds() < timeoutS)){
            //robot.telemetry.addData("Path", "Turning. . .");
            //robot.telemetry.update();
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

    public void rampTurn(double degrees, double power, double timeoutS) {
        planner = new Planner(getIMUAngle(), getIMUAngle()+degrees, power);
        int lFTarget = 0;
        int lBTarget = 0;
        int rFTarget = 0;
        int rBTarget = 0;
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
            

        }

        robot.telemetry.addData("Path", "Turning %.2f degrees", degrees);
        robot.telemetry.update();
        runtime.reset();


        while ((Math.abs(lFTarget - leftFront.getCurrentPosition())>3) && (Math.abs(lBTarget - leftBack.getCurrentPosition())>3) &&
                (Math.abs(rFTarget - rightFront.getCurrentPosition())>3) && (Math.abs(rBTarget - rightBack.getCurrentPosition())>3)
                && robot.opModeIsActive() && (runtime.seconds() < timeoutS)){
            power = planner.getPower(getIMUAngle());
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            robot.telemetry.addData("Ramp turn", "Power: %.2f", power);
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

    public void sideDrive(double distance, double dir, double timeoutS) {//Positive is to the left
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
            runtime.reset();

            while (robot.opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()&& (runtime.seconds() < timeoutS)){
                //robot.telemetry.addData("Path", "Crab Walking. . .");
                //robot.telemetry.update();
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

    public void driveTo(RobotInfo r, Position p) {
        driveTo(r, p, 1);
    }

    public void driveTo(RobotInfo r, Position p, double power) {
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


        turn(turn, power, 1.64);
        simpleDrive(distance, power);

        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
        IMUTurned = getIMUField();
        r.x += Math.cos(Math.toRadians(IMUTurned))*distance;
        r.y += Math.sin(Math.toRadians(IMUTurned))*distance;

        //r.x = p.x;
        //r.y = p.y;
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

    public void quickDrive(RobotInfo r, Position p) {
        quickDrive(r, p, 0.8, 3);
    }

    public void quickDrive(RobotInfo r, Position p, double timeOut, double sideTimeOut/*, boolean brake*/) {
        String tag = "Quick Drive";
        RobotLog.ii(tag, "Start Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);
        RobotLog.ii(tag, "Target Pos: %.2f, %.2f", p.x, p.y);
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;
        double turnBack;
        double turnSide;

        double startIMUangle = getIMUAngle();//for logging
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
            if(turn - turnSide > 0){
                distance = -distance;
            }
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


        turn(turn, 1, timeOut);
        if(Math.abs(turnSide) < 45 && distance<0) {//Move left
            sideDrive(distance, 1, sideTimeOut);
            IMUTurned = getIMUField();
            r.x += Math.cos(Math.toRadians(IMUTurned+90))*Math.abs(distance);
            r.y += Math.sin(Math.toRadians(IMUTurned+90))*Math.abs(distance);
        } else if(Math.abs(turnSide) < 45) {//Move right
            sideDrive(distance, 1, sideTimeOut);
            IMUTurned = getIMUField();
            r.x += Math.cos(Math.toRadians(IMUTurned-90))*Math.abs(distance);
            r.y += Math.sin(Math.toRadians(IMUTurned-90))*Math.abs(distance);
        } else if(Math.abs(turnBack) < 45) {
            simpleDrive(distance, 1);
            IMUTurned = getIMUField();
            r.x -= Math.cos(Math.toRadians(IMUTurned))*Math.abs(distance);
            r.y -= Math.sin(Math.toRadians(IMUTurned))*Math.abs(distance);
        } else {
            simpleDrive(distance, 1);//was 0.7
            IMUTurned = getIMUField();
            r.x += Math.cos(Math.toRadians(IMUTurned))*Math.abs(distance);
            r.y += Math.sin(Math.toRadians(IMUTurned))*Math.abs(distance);
        }

        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
        //IMUTurned = getIMUField();

        //r.x = p.x;
        //r.y = p.y;
        r.degrees = getIMUField();
        RobotLog.ii(tag, "Distance: %.2f", distance);
        RobotLog.ii(tag, "End Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);

        /*telemetry.addData("RobotX:", r.x);
        telemetry.addData("RobotY", r.y);
        telemetry.addData("Robot Heading", r.degrees);*/
        //RobotLog.ii(Tag, "DriveTo: turned IMU angle: %.2f", getIMUAngle() - startIMUangle);
    }

    public void hardDrive(RobotInfo r, Position p, double timeOut, double sideTimeOut, double factor/*, boolean brake*/) {
        String tag = "Quick Drive";
        RobotLog.ii(tag, "Start Pos: (%.2f, %.2f), (%.2f)", r.x, r.y, r.degrees);
        RobotLog.ii(tag, "Target Pos: %.2f, %.2f", p.x, p.y);
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;
        double turnBack;
        double turnSide;

        double startIMUangle = getIMUAngle();//for logging
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
            if(turn - turnSide > 0){
                distance = -distance;
            }
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


        turn(turn*factor, 1, timeOut);
        if(Math.abs(turnSide) < 45 && distance<0) {//Move left
            sideDrive(distance*factor, 1, sideTimeOut);
            IMUTurned = getIMUField();
            r.x += Math.cos(Math.toRadians(IMUTurned+90))*Math.abs(distance);
            r.y += Math.sin(Math.toRadians(IMUTurned+90))*Math.abs(distance);
        } else if(Math.abs(turnSide) < 45) {//Move right
            sideDrive(distance*factor, 1, sideTimeOut);
            IMUTurned = getIMUField();
            r.x += Math.cos(Math.toRadians(IMUTurned-90))*Math.abs(distance);
            r.y += Math.sin(Math.toRadians(IMUTurned-90))*Math.abs(distance);
        } else if(Math.abs(turnBack) < 45) {
            simpleDrive(distance*factor, 1);
            IMUTurned = getIMUField();
            r.x -= Math.cos(Math.toRadians(IMUTurned))*Math.abs(distance);
            r.y -= Math.sin(Math.toRadians(IMUTurned))*Math.abs(distance);
        } else {
            simpleDrive(distance*factor, 1);//was 0.7
            IMUTurned = getIMUField();
            r.x += Math.cos(Math.toRadians(IMUTurned))*Math.abs(distance);
            r.y += Math.sin(Math.toRadians(IMUTurned))*Math.abs(distance);
        }

        /*if(brake == true) {
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
        //IMUTurned = getIMUField();

        //r.x = p.x;
        //r.y = p.y;
        r.degrees = getIMUField();
        RobotLog.ii(tag, "Distance: %.2f", distance);
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


        turn(turn, 0.9, 2.4);

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

    public void turnAcurrate(RobotInfo r, double degrees) {
        String tag = "Turn Accurate";
        RobotLog.ii(tag, "Start Pos: %.2f", r.degrees);
        turn(degrees, 1, 5);
        r.degrees = getIMUField();
        RobotLog.ii(tag, "End Pos: %.2f", r.degrees);
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
