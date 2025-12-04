package org.firstinspires.ftc.teamcode.auto;





import android.graphics.Color;

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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class NutralBluetTest extends LinearOpMode {
    public boolean touchval = false;

    //still writing.

    public void initHardware() {
        initRevolver(13);
        initShooter();
        initArmOne();
        initintake();
        initColor();
        initTouch();

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

    public void initColor(){
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "sensor");
        sensor.setGain(12);
    }

    public void initTouch() {
        TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch");
        if (touch.getValue() == 1) {
            touchval = true;
        } else touchval = false;
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
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

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
                        .stopAndAdd(new CRevo(revolver, sensor, 1))
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

   public class CRevo implements Action {
        private boolean inited = false;
        ElapsedTime timer;
        DcMotorEx revolver;
        NormalizedColorSensor sensor;
        public int ball = 1;

        public CRevo(DcMotorEx r, NormalizedColorSensor cs, int ball) {
            this.ball = ball;
            this.sensor = cs;
            this.revolver = r;
        }

       @Override
       public boolean run(@NonNull TelemetryPacket telemetryPacket) {
           if (!inited) {
               timer = new ElapsedTime();
               revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               NormalizedRGBA colors = sensor.getNormalizedColors();
               float red = colors.red;
               float green = colors.green;
               float blue = colors.blue;

               // Convert to HSV (NOT HSL!)
               float[] hsv = new float[3];
               Color.RGBToHSV((int) (red * 255), (int) (green * 255), (int) (blue * 255), hsv);

               float hue = hsv[0];        // 0–360
               float sat = hsv[1];        // 0–1
               float val = hsv[2];        // 0–1 (brightness)

               // ----- Color thresholds -----

               // Purple: 250–300
               boolean isPurple =
                       (hue >= 220 && hue <= 245) &&
                               (sat >= 0.27 && sat <= 0.4) && (val >= 0.04 && val <= 0.09);       // allow dark

               // Green: 80–160
               boolean isGreen =
                       (hue >= 140 && hue <= 160) &&
                               (sat >= 0.6 && sat <= 0.8) && (val >= 0.02 && val <= 0.2);

               if (ball == 1) {
                   if(isGreen) {
                       revolver.setPower(0);
                   } else revolver.setPower(0.4);
               }

               if (ball == 2) {
                   if (isPurple) {
                       revolver.setPower(0);
                   } else revolver.setPower(0.4);
               }
               inited = true;
           }
           return timer.seconds() < 0.2;
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

