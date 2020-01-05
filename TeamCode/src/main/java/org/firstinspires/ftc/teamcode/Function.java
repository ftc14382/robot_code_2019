package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Function {
    public DcMotor lifter;
    public DcMotor grabber;
    public Servo foundMover;

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

}
