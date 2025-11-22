package org.firstinspires.ftc.teamcode.Pratice;







import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous
public final class autolime extends LinearOpMode {


    public void initHardware() {
        initRevolver(13);
        initShooter();
        initArmOne();
        initLime();

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

    public void initLime() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void shooreron(){
          DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          shooter.setPower(1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");

        int slot1 = 96;
        int slot2 = 192;
        int slot3 = 384;

        double armDown = 0;
        double armUp = 0.3;

        initHardware();
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(-70)
                        .stopAndAdd(new lime())
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-70, 32), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                        .build());

    }

    public class Revolver implements Action {
        private boolean initialized = false;
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
            return timer.seconds() < 0.1;
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
            return timer.seconds() < 0.1;
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

    public class lime implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        Limelight3A limelight;
        LLResult result = limelight.getLatestResult();
        IMU imu;

        public lime() {
                // Access fiducial result
                    YawPitchRollAngles oriontation = imu.getRobotYawPitchRollAngles();
                    limelight.updateRobotOrientation(oriontation.getYaw());
                    limelight.pipelineSwitch(8);
                    LLResult llResult = limelight.getLatestResult();
                    if (llResult != null && llResult.isValid()) {
                        Pose3D botPose = llResult.getBotpose_MT2();
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        telemetry.addData("BotPose", botPose.toString());
                        telemetry.addData("Yaw", botPose.getOrientation().getYaw());

                        if (llResult.getTx() >= -8) {
                            telemetry.addLine("gay sex");
                        }

            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                if (result != null && result.isValid()) {
                    Pose3D botpose = result.getBotpose();

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        int id = fr.getFiducialId();

                        initialized = true;
                    }
                    return timer.seconds() < 0.1;
                }

            }
            return timer.seconds() < 0.1;
        }
    }
}


