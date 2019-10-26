package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Function {
    public DcMotor lifter;
    public DcMotor grabber;

    public void init(HardwareMap ahwMap) {
        lifter = ahwMap.get(DcMotor.class, "lifter");
        grabber = ahwMap.get(DcMotor.class, "grabber");

        lifter.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(DcMotor.Direction.FORWARD);

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
