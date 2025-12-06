package org.firstinspires.ftc.teamcode.Pratice;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;
import java.util.List;

@Disabled
@Config
@Autonomous
public class CloseBTest extends LinearOpMode {

    public void initHardware() {
        initRevolver(13);
        initShooter();
        initArmOne();
        initintake();
        initstopper();
        limeinit();
    }

    public void initShooter() {
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setPower(0);
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
        arm.setPosition(0);
    }

    private void initstopper() {
        double initstopper = 0.3;
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);
        stopper.setPosition(initstopper);
    }

    public void initintake() {
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
    }

    public void limeinit() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
    }

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ====== Your constants ======
        int slot1 = 96;
        int slot2 = 192;
        int slot3 = 384;

        int g = 192;
        int p1 = 288;
        int p2 = 96;

        double armDown = 0;
        double armUp = 0.15;
        double sp = 0.10;

        double stopperu = 0.3;
        double stopperd = 0.3;

        double in = -1;
        // double out = 1;
        // double dead = 0;

        initHardware();

        telemetry.addLine("Ready - press start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start limelight
        limelight.pipelineSwitch(0);
        limelight.start();

        // 1) Go to X=-70 and face tags (your -75)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Action goToScanPose = drive.actionBuilder(startPose)
                .lineToX(-70)
                .turn(Math.toRadians(-75))
                .build();

        Actions.runBlocking(goToScanPose);

        // 2) Scan for tags (keep polling, don't just read once)
        int chosenId = detectAprilTagId(limelight, 1.0); // scan up to 1.0s

        telemetry.addData("Chosen ID", chosenId);
        telemetry.update();

        // 3) Turn back so your tabs behave like before (heading back to 0)
        Action turnBack = drive.actionBuilder(new Pose2d(-70, 0, Math.toRadians(-75)))
                .turn(Math.toRadians(75))
                .build();
        Actions.runBlocking(turnBack);

        // From here on, the robot is physically at (-70,0,0) in your coordinate system
        Pose2d afterScanPose = new Pose2d(-70, 0, Math.toRadians(0));

        // ====== Build tabs FROM THE REAL CURRENT POSE ======
        TrajectoryActionBuilder tab1 = drive.actionBuilder(afterScanPose)
                // ppg
                .stopAndAdd(new Shooter(shooter, 0.93, 15))
                .waitSeconds(0.5)
                .lineToX(10)
                .turn(Math.toRadians(-20))

                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown)) // shot1

                .stopAndAdd(new Revolver(revolver, slot1))
                .waitSeconds(2)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown)) // slot2
                .waitSeconds(1)

                .stopAndAdd(new Revolver(revolver, slot2))
                .waitSeconds(1.3)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(34, -8), Math.toRadians(-91), (pose2dDual, posePath, v) -> 40)
                .stopAndAdd(new intakeAction(intake, in, 10))
                .stopAndAdd(new stopperAction(stopper, stopperd))
                .lineToX(33.4)
                .waitSeconds(1)
                .stopAndAdd(new stopperAction(stopper, stopperu))
                .strafeToLinearHeading(new Vector2d(14, 1), Math.toRadians(-20), (pose2dDual, posePath, v) -> 40)

                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)

                .stopAndAdd(new Revolver(revolver, slot2))
                .waitSeconds(2.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .lineToX(50);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(afterScanPose)
                // pgp
                .stopAndAdd(new Shooter(shooter, 0.93, 15))
                .waitSeconds(0.7)
                .lineToX(10)
                .turn(Math.toRadians(-20))

                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown)) // shot1

                .stopAndAdd(new Revolver(revolver, slot2))
                .waitSeconds(2)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .stopAndAdd(new Revolver(revolver, slot3))
                .waitSeconds(1.3)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(34, -8), Math.toRadians(-91), (pose2dDual, posePath, v) -> 40)
                .stopAndAdd(new intakeAction(intake, in, 10))
                .stopAndAdd(new stopperAction(stopper, stopperd))
                .lineToX(33.4)
                .waitSeconds(1)
                .stopAndAdd(new stopperAction(stopper, stopperu))
                .strafeToLinearHeading(new Vector2d(16, 1), Math.toRadians(-20), (pose2dDual, posePath, v) -> 40)

                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)

                .stopAndAdd(new Revolver(revolver, slot2))
                .waitSeconds(2.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .lineToX(50);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(afterScanPose)
                // gpp
                .stopAndAdd(new Shooter(shooter, 0.93, 15))
                .stopAndAdd(new Revolver(revolver, g))
                .waitSeconds(0.5)
                .lineToX(10)
                .turn(Math.toRadians(-20))

                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)

                .stopAndAdd(new Revolver(revolver, p1))
                .waitSeconds(2.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .stopAndAdd(new Revolver(revolver, p2))
                .waitSeconds(1.6)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(34, -8), Math.toRadians(-91), (pose2dDual, posePath, v) -> 40)
                .stopAndAdd(new intakeAction(intake, in, 10))
                .stopAndAdd(new stopperAction(stopper, stopperd))
                .lineToX(33.4)
                .waitSeconds(1)
                .stopAndAdd(new stopperAction(stopper, stopperu))
                .strafeToLinearHeading(new Vector2d(16, 1), Math.toRadians(-20), (pose2dDual, posePath, v) -> 40)

                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)

                .waitSeconds(1)
                .lineToX(50);

        // ====== Choose ONE trajectory (don’t loop and run multiple) ======
        Action chosen;
        Action closeOut;

        if (chosenId == 23) {
            chosen = tab1.build();
            closeOut = tab1.endTrajectory().fresh().lineToX(-70).build();
        } else if (chosenId == 22) {
            chosen = tab2.build();
            closeOut = tab2.endTrajectory().fresh().lineToX(-70).build();
        } else if (chosenId == 21) {
            chosen = tab3.build();
            closeOut = tab3.endTrajectory().fresh().lineToX(-70).build();
        } else {
            // If nothing found, pick a default so it DOES something predictable
            telemetry.addLine("No tag detected - defaulting to ID 23 (tab1)");
            telemetry.update();
            chosen = tab1.build();
            closeOut = tab1.endTrajectory().fresh().lineToX(-70).build();
        }

        // ====== Run chosen then closeOut (only once) ======
        Actions.runBlocking(new SequentialAction(chosen, closeOut));
    }

    /**
     * Scan for AprilTags for up to timeoutSeconds, then pick the "best" tag among 21/22/23.
     * Best = closest to center (smallest abs X degrees), as a simple stable heuristic.
     */
    private int detectAprilTagId(Limelight3A limelight, double timeoutSeconds) {
        ElapsedTime t = new ElapsedTime();
        int bestId = -1;
        double bestScore = 1e9;

        while (opModeIsActive() && t.seconds() < timeoutSeconds) {
            LLResult r = limelight.getLatestResult();

            if (r != null && r.isValid()) {
                List<LLResultTypes.FiducialResult> frs = r.getFiducialResults();
                if (frs != null) {
                    for (LLResultTypes.FiducialResult fr : frs) {
                        int id = fr.getFiducialId();
                        if (id == 21 || id == 22 || id == 23) {
                            double x = fr.getTargetXDegrees();
                            double y = fr.getTargetYDegrees();
                            double score = Math.abs(x) + (0.3 * Math.abs(y)); // prefer centered

                            if (score < bestScore) {
                                bestScore = score;
                                bestId = id;
                            }
                        }
                    }
                }
            }

            telemetry.addData("ScanTime", "%.2f", t.seconds());
            telemetry.addData("BestIdSoFar", bestId);
            telemetry.addData("BestScore", bestScore);
            telemetry.update();

            idle();
        }
        return bestId;
    }

    // ===============================
    // ===== Custom Action Classes ===
    // ===============================

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
            return timer.seconds() < 0.1; // keep your behavior
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
            return timer.seconds() < 0.1; // keep your behavior
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
            return timer.seconds() < 0.1; // keep your behavior
        }
    }

    public class stopperAction implements Action {
        private boolean initialized = false;
        ElapsedTime timer;
        Servo stopper;
        double stopperpos;

        public stopperAction(Servo s, double position) {
            this.stopper = s;
            this.stopperpos = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer = new ElapsedTime();
                stopper.setPosition(stopperpos);
                initialized = true;
            }
            return timer.seconds() < 0.1; // keep your behavior
        }
    }

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
            return timer.seconds() < 0.1; // keep your behavior
        }
    }
}
