package org.firstinspires.ftc.teamcode.Pratice;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config//important
@Disabled
@TeleOp
public class g extends OpMode {
    private PIDFController controller;//important

    public static double p = 0.1, i = 0, d= 0;
    public static double f = 0.000001;

    public static int target = 57;
    public static int target2 = 249;
    public static int target3 = 155;

    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    private DcMotorEx Revolver;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//allow to do stuff in dash board
        Revolver = hardwareMap.get(DcMotorEx.class,"revolver");

    }

    @Override
    public void loop(){
        if (gamepad1.a) {
            controller.setPIDF(p, i, d, f);
            int revpose = Revolver.getCurrentPosition();
            double pid = controller.calculate(revpose, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;//math that sets the power
            Revolver.setPower(power);
            telemetry.addData("pose1",revpose);
        }
        if (gamepad1.b){
            controller.setPIDF(p, i, d, f);
            int revpose2 = Revolver.getCurrentPosition();
            double pid2 = controller.calculate(revpose2, target2);
            double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f;

            double power = pid2 + ff2;
            Revolver.setPower(power);
            telemetry.addData("pose2",revpose2);
        }

        if (gamepad1.x){
            controller.setPIDF(p, i, d, f);
            int revpose3 = Revolver.getCurrentPosition();
            double pid2 = controller.calculate(revpose3, target3);
            double ff2 = Math.cos(Math.toRadians(target3 / ticks_in_degree)) * f;

            double power = pid2 + ff2;
            Revolver.setPower(power);
            telemetry.addData("pose3",revpose3);
        }


        telemetry.addData("Target",target);
        telemetry.update();

    }
}

