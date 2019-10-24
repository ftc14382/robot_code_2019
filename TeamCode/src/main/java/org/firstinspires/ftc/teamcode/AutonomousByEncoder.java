package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous by Encoder", group="Linear OpMode")
public class AutonomousByEncoder extends LinearOpMode{
    public Mecanum chassis;
    @Override
    public void runOpMode() {
        chassis = new Mecanum();
        chassis.init(hardwareMap, this);
        telemetry.addData(":)", "Initialized");
        telemetry.update();

        telemetry.addData("Ready", "Starting at %7d :%7d",
                chassis.leftFront.getCurrentPosition(),
                chassis.rightFront.getCurrentPosition());
        telemetry.update();

        waitForStart();

        chassis.simpleDrive(12, 1);
        chassis.turn(90, 1);
        chassis.sideDrive(12, 1);
        //encoderDrive(36.0, 36.0, 30.0);
    }

}


