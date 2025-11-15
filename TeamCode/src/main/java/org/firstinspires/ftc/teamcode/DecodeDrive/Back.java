package org.firstinspires.ftc.teamcode.DecodeDrive;









import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Disabled
@Autonomous
public final class Back extends LinearOpMode {
    public void initHardware() {
        initRevolver();
        initShooter();
        initArmOne();

    }

    public void initShooter() {
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initRevolver() {
        int revinit = 96;
        DcMotor revolver = hardwareMap.get(DcMotor.class, "revolver");
        revolver.setDirection(DcMotor.Direction.FORWARD);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setPower(0);
        revolver.setTargetPosition(revinit);
        revolver.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void initArmOne() {
        Servo arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.REVERSE);
        double arminit = 1;
        arm.setPosition(arminit);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap , new Pose2d(0,0,0));

        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotor revolver = hardwareMap.get(DcMotor.class, "revolver");

        int slot1 =96;
        int slot2 =195;
        int slot3 =288;

        double armDown = 1;
        double armUp = 0.8;

        initHardware();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .stopAndAdd(new Shooter(shooter,1,10))
                        .stopAndAdd(new armAction(arm,armUp))
                        .stopAndAdd(new armAction(arm,armDown))
                        .stopAndAdd(new Revolver(revolver,slot1))
                        .stopAndAdd(new armAction(arm,armUp))
                        .stopAndAdd(new armAction(arm,armDown))
                        .stopAndAdd(new Revolver(revolver,slot2))
                        .stopAndAdd(new armAction(arm,armUp))
                        .stopAndAdd(new armAction(arm,armDown))
                        .lineToX(30)
                        .build());

    }

    public class Revolver implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        DcMotor revolver;
        int revopos;



        public Revolver(DcMotor r, int position) {
            this.revolver = r;
            this.revopos = position;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                revolver.setTargetPosition(revopos);
                revolver.setPower(0.5);
                initialized = true;
            }
            return timer.seconds() < 3;
        }


    }

    public class Shooter implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        DcMotor shooter;
        double power;
        double seconds;

        public Shooter(DcMotor s, double w, double seconds) {
            this.shooter = s;
            this.power = w;
            this.seconds = seconds;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                shooter.setPower(power);
                initialized = true;
            }
            return timer.seconds() < 3;
        }


    }


    public class armAction implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        Servo arm;
        double armPos;

        public armAction(Servo s, double position) {
            this.arm = s;
            this.armPos = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                arm.setPosition(armPos);
                initialized = true;
            }

            return timer.seconds() < 0.1;
        }

    }



}







