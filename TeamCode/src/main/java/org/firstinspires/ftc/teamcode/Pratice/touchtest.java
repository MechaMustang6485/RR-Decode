package org.firstinspires.ftc.teamcode.Pratice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class touchtest extends LinearOpMode {


    private DcMotorEx revolver;
    private Servo stopper;
    private TouchSensor touch;
    public boolean touched = false;

    public void initHardware(){
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        stopper = hardwareMap.get(Servo.class, "stopper");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        if (touch.getValue() == 1) {
            touched = true;
        } else touched = false;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Touched?", touched);
            telemetry.addData("1", touch.getValue());
            telemetry.addData("2", touch.toString());
            telemetry.update();
            if (touch.getValue() == 1) {
                revolver.setPower(0);

            } else {

                revolver.setPower(0.4);
            }

        }
    }
}
