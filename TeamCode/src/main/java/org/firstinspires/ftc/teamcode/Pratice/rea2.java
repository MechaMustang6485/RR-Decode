package org.firstinspires.ftc.teamcode.Pratice;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;


@Disabled
@Config
@TeleOp
public class rea2 extends LinearOpMode {
    private PIDFController controller;//important

    GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    public static double p = 0.003, i = 0, d= 0.1;
    public static double f = 0.001;

    public static int target = 6;
    private final double ticks_in_degree = 700/ 180.0;

    private DcMotorEx fl;
    private DcMotorEx br;
    private DcMotorEx fr;
    private DcMotorEx bl;






    public void initHardware() {
        intpin();
        DrivePIDF();
    }

    public void intpin(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        configurePinpoint();
    }
    public void DrivePIDF() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fl = hardwareMap.get(DcMotorEx.class, "Fl");
        bl = hardwareMap.get(DcMotorEx.class, "Bl");
        br = hardwareMap.get(DcMotorEx.class, "Br");
        fr = hardwareMap.get(DcMotorEx.class, "Fr");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {


        }
        waitForStart();
        while (opModeIsActive()) {

            TeleopControls();

        }


    }



    public void configurePinpoint(){
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);


        pinpoint.resetPosAndIMU();

    }

    public void TeleopControls() {
        telemetry.addLine("Push your robot around to see it track");
        telemetry.addLine("Press A to reset the position");
        if(gamepad1.a){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));



        controller.setPIDF(p,i,d,f);
        int flpose = fl.getCurrentPosition();
        int blpose = bl.getCurrentPosition();
        int brpose = br.getCurrentPosition();
        int frpose = fr.getCurrentPosition();
        double Ypo = pose2D.getY(DistanceUnit.INCH);

        double pid = controller.calculate(Ypo, target);

        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid + ff;
        fl.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        fr.setPower(power);


        telemetry.addData("pose",flpose);
        telemetry.addData("pose",blpose);
        telemetry.addData("pose",brpose);
        telemetry.addData("pose",frpose);
        telemetry.addData("Target",target);
        telemetry.update();
    }


}