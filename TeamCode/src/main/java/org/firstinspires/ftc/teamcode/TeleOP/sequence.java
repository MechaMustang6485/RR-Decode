package org.firstinspires.ftc.teamcode.TeleOP;

import static android.os.SystemClock.sleep;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Pratice.please;
import org.firstinspires.ftc.teamcode.auto.NutralBlue;

@TeleOp
public class sequence extends OpMode {

    public DcMotorEx revolver;
    public DcMotor shooter;
    public Servo arm;
    public Servo stopper;
    public NormalizedColorSensor sensor;
    public Limelight3A limelight;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    double driveTrainPower = 0.8;
    public int BallCount = 0;
    public boolean revolverpos = false;


    private PIDFController controller;

    public static double p = 0.1, i = 0, d= 0;
    public static double f = 0.000001;

    public static int target = 57;
    public static int target2 = 249;
    public static int target3 = 155;
    public int targetchoice = 0;

    private final double ticks_in_degree = 700/ 180.0;

    @Override
    public void init() {
        revolver=hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(0);
        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);
        stopper.setPosition(-0.3);
        sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        sensor.setGain(12);
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.setPollRateHz(100);
        controller.setPIDF(p, i, d, f);
        leftFront = hardwareMap.get(DcMotor.class, "Fl"); // Control Hub Motor 3
        rightFront = hardwareMap.get(DcMotor.class, "Fr"); // Control Hub Motor 1
        rightBack = hardwareMap.get(DcMotor.class, "Br"); // Control Hub Motor 0
        leftBack = hardwareMap.get(DcMotor.class, "Bl"); // Control Hub Motor 2

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        limelight.start();
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

    public void RevolverunToPosition(int position){
        revolver.setTargetPosition(position);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setPower(1.0);
    }

    @Override
    public void loop() {
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

        imuDriveInit();

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {

            // pick ONE tag if multiple are visible (priority: 21 > 22 > 23)
            LLResultTypes.FiducialResult chosen = null;
            for (LLResultTypes.FiducialResult fr : llResult.getFiducialResults()) {
                int id = fr.getFiducialId();
                if (id == 21) { chosen = fr; break; }
                if (id == 22 && chosen == null) chosen = fr;
                if (id == 23 && chosen == null) chosen = fr;

            }


            if (chosen != null) {
                switch (chosen.getFiducialId()) {
                    case 21:
                        limelight.stop();
                        shooter.setPower(0.9);
                        RevolverunToPosition(96);
                        arm.setPosition(0.17);
                        arm.setPosition(0);
                        RevolverunToPosition(192);
                        arm.setPosition(0.17);
                        arm.setPosition(0);
                        RevolverunToPosition(288);
                        arm.setPosition(0.17);
                        arm.setPosition(0);
                        shooter.setPower(0);
                        break;
                    case 22:
                        telemetry.addLine("PGP");
                        break;
                    case 23:
                        telemetry.addLine("PPG");
                        break;
                    default:
                        // ignore other IDs
                        break;
                }



            }
        }

        if (BallCount != 0) {
            switch (BallCount) {
                case 1:

            }
        }


    }
}
