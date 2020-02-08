package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Function {
    public DcMotor lifter;
    public DcMotor grabber;
    public Servo foundMover;
    public LinearOpMode robot;
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        lifter = ahwMap.get(DcMotor.class, "lifter");
        grabber = ahwMap.get(DcMotor.class, "grabber");
        foundMover = ahwMap.get(Servo.class, "found_servo");

        lifter.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(DcMotor.Direction.FORWARD);

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foundMover.setPosition(1);
    }

    public void liftTo(int position, double timeoutS) {
        lifter.setTargetPosition(position);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(1);
        while (lifter.isBusy() && (runtime.seconds() < timeoutS) && robot.opModeIsActive()){}
        lifter.setPower(0);
    }

}
