package org.firstinspires.ftc.teamcode.Pratice;






import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * Control Hub Configuration
 *
 * Drive Train:
 * Motor Port 0: = rightBack
 * Motor Port 1: = rightFront
 * Motor Port 2 = leftBack
 * Motor Port 3 = leftFront
 *
 * Servos:
 * Servo Port 0 = arm
 * Servo Port 1 = wrist
 * Servo Port 2 = intake
 *
 * Expansion Hub Configuration:
 *
 * Motors:
 * Motor Port 0 = slides
 * Motor Port 1 = hang2
 *
 */

/**
 * Controller inputs
 * Gamepad 1:
 *
 * Button A = Drive Train Fast
 * Button B = Arm Neutral
 * Button X = Drive Train Slow
 * Button Y = Reset Slide Encoders
 *
 * Dpad Left = Score High Chamber
 * Dpad Up = High Bucket
 * Dpad Right = High Chamber
 * Dpad Down =  Zero Position Slides
 *
 * Bumper Left = Arm Down
 * Bumper Right = Arm Up
 *
 * Both Triggers = nothing atm
 *
 * Gamepad 2:
 *
 * Button A = Wrist Position Down also Starting position
 * Button B = Wrist Position Right
 * Button X = Wrist Position Left
 * Button Y = Wrist Position Up
 *
 * Dpad Left = Slide Override Down
 * Dpad Up = Arm Override Up
 * Dpad Right = Slide Override Up
 * Dpad Down = Arm Override Down
 *
 * Right bumper = Stop intake Servo
 *
 * = Wrist Override Left (NOT USED)
 * = Wrist Override Right (NOT USED)
 *
 * Left + Right Bumpers = Level 2 Climb
 * Trigger Left = Intake
 * Trigger Right = Outtake
 *
 */

//@Disabled
@Config
@TeleOp(group = "Primary")
public class bean extends LinearOpMode {

    DcMotor leftFront;
    //private double frontLeftSensitivity = 0.5;
    DcMotor rightFront;
    //private double frontRightSensitivity = 0.5;
    DcMotor rightBack;
    //private double backRightSensitivity = 0.5;
    DcMotor leftBack;
    //private double backLeftSensitivity = 0.5;
    double driveTrainPower = 0.8;
    IMU imu;
    private DcMotor shooter;
    private CRServo intake;
    private Servo arm;
    private Limelight3A limelight;
    private Servo stopper;
    private DcMotorEx revolver;
    private NormalizedColorSensor sensor;
    private TouchSensor touch;

    private PIDFController controller;

    public static double p = 0.1, i = 0, d= 0;
    public static double f = 0.000001;

    public static int target = 96;
    public static int target2 = 192;
    public static int target3 = 284;

    private final double ticks_in_degree = 700/ 180.0;

    private double armpos = 0;
    private double armshootpos = 0.15;
    private int revolverpospos = 96;
    private boolean touchVal = false;
    private double rapidcount = 0;

    //HARDWARE
    public void initHardware() {
        initServo();
        initShooter();
        initLimeLight();
        initarm();
        initstopper();
        initRevolver();
        driveTrain();
        initcs();
        initTouch();
    }

    public void driveTrain() {
        leftFront = hardwareMap.get(DcMotor.class, "Fl"); // Control Hub Motor 3
        rightFront = hardwareMap.get(DcMotor.class, "Fr"); // Control Hub Motor 1
        rightBack = hardwareMap.get(DcMotor.class, "Br"); // Control Hub Motor 0
        leftBack = hardwareMap.get(DcMotor.class, "Bl"); // Control Hub Motor 2

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initcs() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        sensor.setGain(12);
    }

    public void initTouch(){
        touch = hardwareMap.get(TouchSensor.class, "touch");
        if (touch.getValue() == 1) {
            touchVal = true;
        } else touchVal = false;
    }

    private void initShooter() {
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void initRevolver(){
        revolver=hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initarm() {
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(armpos);
    }

    private void initstopper() {
        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);
        stopper.setPosition(-0.3);
    }

    public void initLimeLight(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    private void initServo() {
        intake=hardwareMap.get(CRServo.class,"intake");
        intake.setPower(0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void runOpMode() {

        initHardware();
        while (!isStarted()) {

        }
        waitForStart();
        while (opModeIsActive()) {
            imuDriveInit();
            TeleOpControls();
            slotTelemetry();
            telemetry.update();
        }
    }

    public void imuDriveInit() {
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        double leftJoyStickXAxis = gamepad1.left_stick_x * 1.1; //1.1 use to counteract imperfect strafing
        double leftJoyStickYAxis = -gamepad1.left_stick_y; //y stick value is reversed
        double rightJoyStickXAxis = gamepad1.right_stick_x;
        double botOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotateX = leftJoyStickXAxis * Math.cos(-botOrientation) - leftJoyStickYAxis * Math.sin(-botOrientation);
        double rotateY = leftJoyStickXAxis * Math.sin(-botOrientation) + leftJoyStickYAxis * Math.cos(-botOrientation);

        rotateX = rotateX * 1.1;  // Counteract imperfect strafing
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(leftJoyStickYAxis) + Math.abs(leftJoyStickXAxis) + Math.abs(rightJoyStickXAxis), 1);
        double frontLeftMotorPower = (rotateY + rotateX + rightJoyStickXAxis) / denominator;
        double backLeftMotorPower = (rotateY - rotateX + rightJoyStickXAxis) / denominator;
        double frontRightMotorPower = (rotateY - rotateX - rightJoyStickXAxis) / denominator;
        double backRightMotorPower = (rotateY + rotateX - rightJoyStickXAxis) / denominator;

        //Set motor power
        leftFront.setPower(frontLeftMotorPower * driveTrainPower);
        leftBack.setPower(backLeftMotorPower * driveTrainPower);
        rightBack.setPower(backRightMotorPower * driveTrainPower);
        rightFront.setPower(frontRightMotorPower * driveTrainPower);


        //IMU Reset
        if (gamepad1.back) {
            imu.resetYaw();
        }
    }

    private void TeleOpControls() {
        // Read normalized color data
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

        // Yellow: 40–80
        boolean isYellow =
                (hue > 40 && hue < 80);

        // White: low saturation + high brightness
        boolean isWhite =
                sat < 0.2 && val > 0.5;


        if (gamepad2.leftBumperWasPressed()) {
            intake.setPower(1);
        }

        if (gamepad2.left_trigger >= 1) {
            stopper.setPosition(-0.3);
            intake.setPower(-1);
        }

        if (gamepad2.dpad_left) {
            intake.setPower(0);
        }

        if (gamepad2.right_bumper) {
            stopper.setPosition(0.3);
            sleep(20);
            shooter.setPower(1);
        }

        if (gamepad2.dpadUpWasPressed()) {
            arm.setPosition(armshootpos);
            sleep(500);
            arm.setPosition(armpos);
        }


        if (gamepad2.right_trigger >= 1) {
            stopper.setPosition(-0.3);
            sleep(10);
            shooter.setPower(0);
        }

        if (gamepad2.x) {
            /* controller.setPIDF(p, i, d, f);
            int revpose = revolver.getCurrentPosition();
            double pid = controller.calculate(revpose, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;
            revolver.setPower(power);
            telemetry.addData("pose1",revpose); */
            double system = 1;

            if (system == 1) {
                Revolvernextslot(1);
                system = 2;
            } else if (system == 2) {
                Revolvernextslot(2);
                system = 3;
            } else if (system == 3) {
                Revolvernextslot(3);
                system = 1;
            }
        }//slot 1: 96, slot 2: 192, slot 3: 288


        if(gamepad2.y) {
            controller.setPIDF(p, i, d, f);
            int revpose3 = revolver.getCurrentPosition();
            double pid2 = controller.calculate(revpose3, target3);
            double ff2 = Math.cos(Math.toRadians(target3 / ticks_in_degree)) * f;

            double power = pid2 + ff2;
            revolver.setPower(power);
        }

        if (shooter.getPower() >= 0.1) {
            intake.setPower(0);
            stopper.setPosition(0.3);
        }

        if (intake.getPower() >= 0.1) {
            shooter.setPower(0);
            stopper.setPosition(-0.3);
        }

/*
        if (gamepad2.aWasPressed()) {
            if(!touchVal && !isGreen) {
                revolver.setPower(0.4);
            } else if (touchVal && isGreen && !isPurple) {
                revolver.setPower(0);
            }
        }

 */

        if(gamepad2.b) {//use to be bWasPressed
            controller.setPIDF(p, i, d, f);
            int revpose2 = revolver.getCurrentPosition();
            double pid2 = controller.calculate(revpose2, target2);
            double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f;

            double power = pid2 + ff2;
            revolver.setPower(power);

            /*
            if (!touchVal && !isPurple) {
                revolver.setPower(0.4);
            } else if (touchVal && isPurple || !isGreen) {
                revolver.setPower(0);
            }

             */
        }

        if (gamepad2.dpadRightWasPressed()) {
            stopper.setPosition(0.3);
            revolver.setPower(0.5);
            shooter.setPower(1);

            /*
            if (rapidcount == 3) {
                rapidcount = 0;
                stopper.setPosition(-0.3);
                //RevolverunToPosition(96);
                revolverpospos = 96;
                shooter.setPower(0);
                arm.setPosition(armpos);
            }

            if (isPurple || isGreen) {
                arm.setPosition(armshootpos);
                arm.setPosition(armshootpos);
                rapidcount = rapidcount + 1;
            }

             */
        }


    }

    public void Revolvernextslot(int slot){
        /* controller.setPIDF(p, i, d, f);
        int revpose = revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        revolver.setPower(power);
        telemetry.addData("pose1",revpose);

        controller.setPIDF(p, i, d, f);
        int revpose3 = revolver.getCurrentPosition();

        double power = pid3 + ff2;
        revolver.setPower(power);


        controller.setPIDF(p, i, d, f);
        int revpose2 = revolver.getCurrentPosition();
        double pid2 = controller.calculate(revpose2, target2);
        double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f;

        double power = pid2 + ff2;
        revolver.setPower(power); */

        double pids = 0;
        double ffs = 0;

        controller.setPIDF(p, i, d, f);
        int revpose = revolver.getCurrentPosition();
        double pid = controller.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;


        double pid2 = controller.calculate(revpose, target2);
        double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree)) * f;


        double pid3 = controller.calculate(revpose, target3);
        double ff3 = Math.cos(Math.toRadians(target3 / ticks_in_degree)) * f;

        if(slot == 1){
            pids = pid;
            ffs = ff;
        } else if (slot == 2) {
            pids = pid2;
            ffs = ff2;
        } else if (slot == 3) {
            pids = pid3;
            ffs = ff3;
        }

        double power = pids + ffs;

        revolver.setPower(power);





    }



    public void slotTelemetry(){
        telemetry.addLine("Left Trigger = Intake");
        telemetry.addLine("Left Bumper = Outtake");
        telemetry.addLine("Dpad Left = Stop Intake/Outtake");
        telemetry.addLine("Dpad Up = Arm Shoot Pos");
        telemetry.addLine("Dpad Down = Arm Down Shoot Pos");
        telemetry.addLine("Right Trigger = Shooter Start");
        telemetry.addLine("Right Bumper = Shooter Stop");
        telemetry.addData("Shooter Power: ", shooter.getPower());
        telemetry.addData("Intake Power: ", intake.getPower());
        telemetry.addData("Stopper Pos: ", stopper.getPosition());
        telemetry.addData("Arm Pos: ", arm.getPosition());
        telemetry.addData("Limelight: ", limelight.getStatus().getPipelineIndex());
        telemetry.update();
    }
}

