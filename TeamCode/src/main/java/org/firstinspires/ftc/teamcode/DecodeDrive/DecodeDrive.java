package org.firstinspires.ftc.teamcode.DecodeDrive;




import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(group = "Primary")
public class DecodeDrive extends LinearOpMode {

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

    private double armpos = 0.6;
    private double armshootpos = 0.3;
    private int revolverpos = 96;

    //HARDWARE
    public void initHardware() {
        initServo();
        initShooter();
        initLimeLight();
        initarm();
        initstopper();
        initRevolver(13);
        driveTrain();
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

    private void initShooter() {
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void initRevolver(double PIDposition){
        revolver=hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPositionPIDFCoefficients(PIDposition);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setPower(0);
        revolver.setTargetPosition(96);
        revolverpos = 96;
        revolver.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
            motortelemetry();
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
            sleep(20);
            arm.setPosition(armshootpos);
        }

        if (gamepad2.dpadDownWasPressed()) {
            arm.setPosition(0);
        }

        if (gamepad2.dpadUpWasPressed()) {
            arm.setPosition(armshootpos);
        }


        if (gamepad2.right_trigger >= 1) {
            stopper.setPosition(-0.3);
            sleep(10);
            arm.setPosition(armpos);
            sleep(10);
            shooter.setPower(0);
        }

        if (gamepad2.dpadRightWasPressed()) {
            stopper.setPosition(0.3);
        }

        if (gamepad2.aWasPressed()) {
            revolver.setPower(0.3);
        }

        if (gamepad2.xWasPressed()) {
            if (revolverpos == 96) {
                RevolverunToPosition(192);
                revolverpos = 192;
            } else if (revolverpos == 192) {
                RevolverunToPosition(288);
                revolverpos = 288;
            } else {
                RevolverunToPosition(96);
                revolverpos = 96;
            }
        }//slot 1: 96, slot 2: 192, slot 3: 288

        if(gamepad2.yWasPressed()) {
            stopper.setPosition(0.3);
            sleep(20);
            shooter.setPower(0.8);
            sleep(20);
            arm.setPosition(armshootpos);
        }

        if (shooter.getPower() >= 0.1) {
            intake.setPower(0);
            stopper.setPosition(0.3);
        }

        if (intake.getPower() >= 0.1) {
            shooter.setPower(0);
            stopper.setPosition(-0.3);
        }

    }

    public void RevolverunToPosition(int position){
        revolver.setTargetPosition(position);
        revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolver.setPower(1.0);

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
        telemetry.update();
    }

    public void motortelemetry() {
        telemetry.addData("leftFront: Position", leftFront.getCurrentPosition());
        telemetry.addData("leftBack: Position", leftBack.getCurrentPosition());
        telemetry.addData("rightFront: Position", rightFront.getCurrentPosition());
        telemetry.addData("rightBack: Position", rightBack.getCurrentPosition());
    }
}
