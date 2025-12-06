package org.firstinspires.ftc.teamcode.Pratice;






import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.NutralBlue;

@Disabled
@Autonomous
public final class please extends LinearOpMode {

    private PIDFController controller;

    public static double p = 0.1, i = 0, d= 0;
    public static double f = 0.000001;

    public static int target = 96;
    public static int target2 = 192;
    public static int target3 = 284;

    private final double ticks_in_degree = 700/ 180.0;


    public void initHardware() {
        initRevolver();


    }

    public void initRevolver() {
        int revinit = 96;
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");

        int slot1 =96;
        int slot2 =192;
        int slot3 = 384;

        double armDown = 0;
        double armUp = 0.15;

        double in = -1;
        double out = 1;
        double dead = 0;

        initHardware();
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        //3 artifact
                        .stopAndAdd(new Revolver(revolver,target))
                        .build());

    }
    //custom action revolver
    public class Revolver implements Action {
        private boolean initialized = false;//don't touch
        ElapsedTime timer;
        DcMotorEx revolver;
        int revpose2 = revolver.getCurrentPosition();
        double pid2 = controller.calculate(revpose2, target2);
        double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f;
        double power = pid2 + ff2;


        public Revolver(DcMotorEx r, int position) {
            controller.setPIDF(p, i, d, f);
            int revpose2 = revolver.getCurrentPosition();
            double pid2 = controller.calculate(revpose2, target2);
            double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f;
            double power = pid2 + ff2;
            revolver.setPower(power);
            this.revolver = r;
            this.pid2 = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();

                revolver.setPower(power);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }


    }


}

