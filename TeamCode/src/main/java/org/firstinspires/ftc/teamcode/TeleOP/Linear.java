package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Linear extends OpMode {

    public DcMotor lift;

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) {
            lift.setPower(1);
        }

        if (gamepad1.dpadUpWasReleased()) {
            lift.setPower(0);
        }

        if (gamepad1.dpadDownWasPressed()) {
            lift.setPower(-1);
        }

        if(gamepad1.dpadDownWasReleased()) {
            lift.setPower(0);
        }
    }
}
