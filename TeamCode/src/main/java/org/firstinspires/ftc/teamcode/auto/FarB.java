package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous
public class FarB extends LinearOpMode {


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
        double arminit = 0;
        arm.setPosition(arminit);

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
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap,  new Pose2d(0, 0, Math.toRadians(0)));

        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        DcMotorEx revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("New Code?");

        int slot1 =96;
        int slot2 =192;
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
        double out = 1;
        double dead = 0;

        initHardware();
        waitForStart();

        limelight.start();
        LLResult llResult = limelight.getLatestResult();


                TrajectoryActionBuilder tab1 = drive.actionBuilder( new Pose2d(0, 0, Math.toRadians(0)))
                        //ppg
                        .stopAndAdd(new Shooter(shooter,0.93,15))
                        .waitSeconds(0.5)
                        .lineToX(10)
                        .turn(Math.toRadians(20))

                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm, armUp))
                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm, armDown))//shot1

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
                        .stopAndAdd(new armAction(arm, armDown))//slot2
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
                        .stopAndAdd(new armAction(arm, armDown))//shot3
                        .waitSeconds(1)

                        .strafeToLinearHeading(new Vector2d(64, 28), Math.toRadians(91), (pose2dDual, posePath, v) -> 40)
                        .stopAndAdd(new intakeAction(intake, in, 10))
                        .stopAndAdd(new stopperAction(stopper,stopperd))
                        // .turn(Math.toRadians(64))
                        .lineToX(63.7)
                        .waitSeconds(1)
                        /*
                        .stopAndAdd(new Revolver(revolver, slot2))
                        .waitSeconds(1)
                        .lineToX(63.5)

                         */
                        .stopAndAdd(new stopperAction(stopper,stopperu))
                        .strafeToLinearHeading(new Vector2d(21, 1), Math.toRadians(20), (pose2dDual, posePath, v) -> 40)

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
                        .stopAndAdd(new armAction(arm, armDown))//slot2
                        .waitSeconds(1)
                        .lineToX(50) ;




                TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        //pgp
                        .stopAndAdd(new Shooter(shooter,0.93,15))
                        .waitSeconds(0.7)
                        .lineToX(10)
                        .turn(Math.toRadians(20))

                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm, armUp))
                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm, armDown))//shot1

                        .stopAndAdd(new Revolver(revolver, slot2))
                        .waitSeconds(2)
                        .stopAndAdd(new armAction(arm, armUp))
                        .waitSeconds(0.5)
                        .stopAndAdd(new armAction(arm, armDown))//slot2
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
                        .stopAndAdd(new armAction(arm, armDown))//shot3
                        .waitSeconds(1)



                        .strafeToLinearHeading(new Vector2d(64, 28), Math.toRadians(91), (pose2dDual, posePath, v) -> 40)
                        .stopAndAdd(new intakeAction(intake, in, 10))
                        .stopAndAdd(new stopperAction(stopper,stopperd))
                        // .turn(Math.toRadians(64))
                        .lineToX(63.7)
                        .waitSeconds(1)
                        /*
                        .stopAndAdd(new Revolver(revolver, slot2))
                        .waitSeconds(1)
                        .lineToX(63.5)

                         */
                        .stopAndAdd(new stopperAction(stopper,stopperu))
                        .strafeToLinearHeading(new Vector2d(21, 1), Math.toRadians(20), (pose2dDual, posePath, v) -> 40)


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
                        .stopAndAdd(new armAction(arm, armDown))//slot2
                        .waitSeconds(1)
                        .lineToX(50) ;


        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(0, 0, 0))

                        //gpp
                        .stopAndAdd(new Shooter(shooter,0.93,15))
                        .stopAndAdd(new Revolver(revolver, g))
                        .waitSeconds(0.5)
                        .lineToX(10)
                        .turn(Math.toRadians(20))

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
                .stopAndAdd(new armAction(arm, armDown))//slot2
                .waitSeconds(1)

                .stopAndAdd(new Revolver(revolver, p2))
                .waitSeconds(1.6)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))//shot3
                .waitSeconds(1)


               // .lineToX(58)
                .strafeToLinearHeading(new Vector2d(64, 28), Math.toRadians(91), (pose2dDual, posePath, v) -> 40)
                .stopAndAdd(new intakeAction(intake, in, 10))
                .stopAndAdd(new stopperAction(stopper,stopperd))
               // .turn(Math.toRadians(64))
                .lineToX(63.7)
                .waitSeconds(1)
                /*
                .stopAndAdd(new Revolver(revolver, slot2))
                .waitSeconds(1)
                .lineToX(63.5)

                 */
                .stopAndAdd(new stopperAction(stopper,stopperu))
                .strafeToLinearHeading(new Vector2d(21, 1), Math.toRadians(20), (pose2dDual, posePath, v) -> 40)

               // .stopAndAdd(new Revolver(revolver, p2))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, sp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)


/*
                .stopAndAdd(new Revolver(revolver, g))
                .waitSeconds(2.5)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.5)
                .stopAndAdd(new armAction(arm, armDown))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armUp))
                .waitSeconds(0.2)
                .stopAndAdd(new armAction(arm, armDown))//slot2

 */
                .waitSeconds(1)
                .lineToX(50);






        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                        .build();


        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                double id = fr.getFiducialId();

                while (!isStopRequested() && !opModeIsActive()) {
                    double position = id;
                    telemetry.addData("Position during Init", position);
                    telemetry.update();
                }

                double startPosition = id;
                telemetry.addData("Starting Position", startPosition);
                telemetry.update();

//                waitForStart();

                if (isStopRequested()) return;

                Action trajectoryActionChosen = null;
                if (id == 23) {
                    trajectoryActionChosen = tab1.build();
                }  if (id == 22) {
                    trajectoryActionChosen = tab2.build();
                } if(id== 21) {
                    trajectoryActionChosen = tab3.build();
                }

                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryActionChosen,
                                trajectoryActionCloseOut
                        )
                );
            }
        }
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