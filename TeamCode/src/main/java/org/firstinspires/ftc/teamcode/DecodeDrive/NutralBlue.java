package org.firstinspires.ftc.teamcode.DecodeDrive;





import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class NutralBlue extends LinearOpMode {


    public void initHardware() {
        initRevolver(13);
        initShooter();
        initArmOne();
        initintake();

    }

    public void initShooter() {
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initRevolver(double PIDposition) {
        int revinit = 96;
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPositionPIDFCoefficients(PIDposition);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setPower(0);
        revolver.setTargetPosition(revinit);
        revolver.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void initArmOne() {
        Servo arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        double arminit = 0;
        arm.setPosition(arminit);
    }

    public void initintake() {
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
    }


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");

        int slot1 =96;
        int slot2 =192;
        int slot3 = 384;

        double armDown = 0;
        double armUp = 0.3;

        double in = -1;
        double out = 1;
        double dead = 0;

        initHardware();
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        //3 artifact
                        .lineToX(-70)
                        .stopAndAdd(new Shooter(shooter,0.76,15))
                        .waitSeconds(1.5)
                        .stopAndAdd(new armAction(arm, armUp))
                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm,armDown))//shot1
                        .waitSeconds(2)
                        .stopAndAdd(new Revolver(revolver,slot1))
                        .waitSeconds(1)
                        .stopAndAdd(new armAction(arm, armUp))
                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm,armDown))//shot2
                        .waitSeconds(2)
                        .stopAndAdd(new Revolver(revolver,slot2))
                        .waitSeconds(2)
                        .stopAndAdd(new armAction(arm, armUp))
                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm,armDown))//shot3
                        .strafeToLinearHeading(new Vector2d(-70,32), Math.toRadians(30), (pose2dDual, posePath, v) -> 40)
                        .stopAndAdd(new Shooter(shooter,0,15))
                        .waitSeconds(0.5)

                        //collect artifacts
                        .stopAndAdd(new intakeAction(intake,in,10))
                        .lineToX(-54)
                        .waitSeconds(0.5)
                        .lineToX(-52)
                        .build());

    }
//custom action revolver
    public class Revolver implements Action {
        private boolean initialized = false;//don't touch
        ElapsedTime timer;
        DcMotorEx revolver;
        int revopos;


        public Revolver(DcMotorEx r, int position) {
            this.revolver = r;
            this.revopos = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                revolver.setTargetPosition(revopos);
                revolver.setPower(1);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }


    }

    //custom action shooter
    public class Shooter implements Action {
        private boolean initialized = false;//don't touch
        ElapsedTime timer;
        DcMotor shooter;
        double power;
        double seconds;

        public Shooter(DcMotor s, double w, double seconds) {
            this.shooter = s;
            this.power = w;
            this.seconds = seconds;//don't touch

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                shooter.setPower(power);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }


    }

    //custom action arm
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

            return timer.seconds() < 0.1;//don't touch
        }

    }
    //custom action intake
    public class intakeAction implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        double seconds;
        CRServo intake;
        double power;

        public intakeAction(CRServo s, double power, double seconds) {
            this.intake = s;
            this.power = power;
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                intake.setPower(power);
                initialized = true;
            }
            return timer.seconds() < 0.1;//don't touch
        }
    }

}

