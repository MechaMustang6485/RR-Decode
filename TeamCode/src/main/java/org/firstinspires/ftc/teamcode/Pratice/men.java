package org.firstinspires.ftc.teamcode.Pratice; // change if needed

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Disabled
@TeleOp(name = "FieldCentric_LL3A_Tag24_NoAdjust", group = "TeleOp")
public class men extends LinearOpMode {

    // ---- drivetrain ----
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // ---- shooter ----
    private DcMotor shooter;

    // ---- sensors ----
    private IMU imu;
    private Limelight3A limelight;

    // ---- drive scaling ----
    private double driveTrainPower = 1.0;

    // ---- Limelight / tag constants ----
    private static final int TARGET_TAG_ID = 24;
    private static final double CLOSE_DISTANCE_METERS = 1.5; // tweak

    @Override
    public void runOpMode() throws InterruptedException {

        // ====== HARDWARE MAP ======

        leftFront  = hardwareMap.get(DcMotor.class, "Fl");
        leftBack   = hardwareMap.get(DcMotor.class, "Bl");
        rightFront = hardwareMap.get(DcMotor.class, "Fr");
        rightBack  = hardwareMap.get(DcMotor.class, "Br");

        shooter    = hardwareMap.get(DcMotor.class, "shooter");

        imu        = hardwareMap.get(IMU.class, "imu");
        limelight  = hardwareMap.get(Limelight3A.class, "limelight"); // name must match RC config

        // Reverse one side if needed (typical for mecanum)
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        // ====== IMU SETUP ======
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParams);

        // ====== LIMELIGHT SETUP ======
        limelight.setPollRateHz(100); // fast updates
        limelight.pipelineSwitch(8);  // make sure pipeline 0 is your AprilTag+MT2 pipeline
        limelight.start();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            imuDriveInit(); // field-centric drive + shooter logic
            telemetry.update();
        }
    }

    /**
     * Field-centric drive + Limelight3A:
     * - Shooter power based on distance to AprilTag 24
     * - NO heading/turn adjustment from AprilTag
     */
    public void imuDriveInit() {

        // --- Get robot yaw for field-centric (rad) + MT2 (deg) ---
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double botOrientationRad = orientation.getYaw(AngleUnit.RADIANS);
        double botOrientationDeg = orientation.getYaw(AngleUnit.DEGREES);

        // Update Limelight with robot yaw (needed for MT2 fusion)
        limelight.updateRobotOrientation(botOrientationDeg);

        double shooterPower = 0.0;

        // === LIMELIGHT PROCESSING (only for shooter + pose, no heading adjust) ===
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // Find AprilTag 24 and compute distance
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult tag24 = null;

            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f.getFiducialId() == TARGET_TAG_ID) {
                        tag24 = f;
                        break;
                    }
                }
            }

            if (tag24 != null) {
                Pose3D robotInTagSpace = tag24.getRobotPoseTargetSpace();
                double x = robotInTagSpace.getPosition().x;
                double y = robotInTagSpace.getPosition().y;
                double z = robotInTagSpace.getPosition().z;

                double distanceMeters = Math.sqrt(x * x + y * y + z * z);

                if (distanceMeters < CLOSE_DISTANCE_METERS) {
                    shooterPower = 0.6;
                } else {
                    shooterPower = 1.0;
                }

                telemetry.addData("Tag24 Dist (m)", distanceMeters);

                // MT2 botpose (if you want field X/Y/heading for later)
                Pose3D botposeMT2 = result.getBotpose_MT2();
                if (botposeMT2 != null) {
                    double fieldX = botposeMT2.getPosition().x;
                    double fieldY = botposeMT2.getPosition().y;

                    // TODO: plug into your pose estimator if/when you make one
                    // double fieldHeadingDeg = botposeMT2.getOrientation().getYaw(AngleUnit.DEGREES);
                    // myPoseEstimator.setPose(fieldX, fieldY, Math.toRadians(fieldHeadingDeg));

                    telemetry.addData("MT2 X", fieldX);
                    telemetry.addData("MT2 Y", fieldY);
                }

            } else {
                telemetry.addLine("No Tag 24 in view");
            }

        } else {
            telemetry.addLine("LL3A: No valid result");
        }

        // Apply shooter power based on distance
        shooter.setPower(shooterPower);

        // === FIELD-CENTRIC DRIVE (your math, untouched except for no headingCorrection) ===

        double leftJoyStickXAxis = gamepad1.left_stick_x * 1.1; // 1.1 to counter imperfect strafing
        double leftJoyStickYAxis = -gamepad1.left_stick_y;      // y is reversed

        // Pure driver turn now, no AprilTag correction
        double rightJoyStickXAxis = gamepad1.right_stick_x;

        // Rotate movement direction counter to robot rotation (field-centric)
        double rotateX = leftJoyStickXAxis * Math.cos(-botOrientationRad)
                - leftJoyStickYAxis * Math.sin(-botOrientationRad);
        double rotateY = leftJoyStickXAxis * Math.sin(-botOrientationRad)
                + leftJoyStickYAxis * Math.cos(-botOrientationRad);

        rotateX = rotateX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(
                Math.abs(leftJoyStickYAxis) + Math.abs(leftJoyStickXAxis) + Math.abs(rightJoyStickXAxis),
                1
        );

        double frontLeftMotorPower  = (rotateY + rotateX + rightJoyStickXAxis) / denominator;
        double backLeftMotorPower   = (rotateY - rotateX + rightJoyStickXAxis) / denominator;
        double frontRightMotorPower = (rotateY - rotateX - rightJoyStickXAxis) / denominator;
        double backRightMotorPower  = (rotateY + rotateX - rightJoyStickXAxis) / denominator;

        leftFront.setPower(frontLeftMotorPower   * driveTrainPower);
        leftBack.setPower(backLeftMotorPower     * driveTrainPower);
        rightBack.setPower(backRightMotorPower   * driveTrainPower);
        rightFront.setPower(frontRightMotorPower * driveTrainPower);

        // IMU yaw reset
        if (gamepad1.back) {
            imu.resetYaw();
        }

        telemetry.addData("Shooter Power", shooterPower);
    }
}
