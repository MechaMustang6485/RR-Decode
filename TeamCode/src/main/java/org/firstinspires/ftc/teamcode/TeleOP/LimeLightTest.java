package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
public class LimeLightTest extends OpMode {
    private Limelight3A limelight;

    private DcMotor Br;
    private DcMotor Bl;
    private DcMotor Fr;
    private DcMotor Fl;
    private DcMotor shooter;

    private IMU imu;
    double driveTrainPower = 0.8;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.setPollRateHz(100);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        Fl = hardwareMap.get(DcMotor.class, "Fl");
        Fr = hardwareMap.get(DcMotor.class, "Fr");
        Br = hardwareMap.get(DcMotor.class, "Br");
        Bl = hardwareMap.get(DcMotor.class, "Bl");
        Fl.setDirection(DcMotor.Direction.REVERSE);
        Fr.setDirection(DcMotor.Direction.FORWARD);
        Br.setDirection(DcMotor.Direction.FORWARD);
        Bl.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {


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
                Fl.setPower(frontLeftMotorPower * driveTrainPower);
                Bl.setPower(backLeftMotorPower * driveTrainPower);
                Br.setPower(backRightMotorPower * driveTrainPower);
                Fr.setPower(frontRightMotorPower * driveTrainPower);


                //IMU Reset
                if (gamepad1.back) {
                    imu.resetYaw();
                }
                if(gamepad1.b){
                    shooter.setPower(0);
                }

                boolean limelightLoopEnabled = false;


        if (gamepad1.aWasPressed()) {
            limelightLoopEnabled = !limelightLoopEnabled;
        }

        if (limelightLoopEnabled){
            YawPitchRollAngles oriontation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(oriontation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                    if (llResult != null && llResult.isValid()) {
                        Pose3D botPose = llResult.getBotpose_MT2();
                        telemetry.addData("BotPose", botPose.toString());
                        telemetry.addData("Yaw", botPose.getOrientation().getYaw());


                        if (llResult.getTy() <= -4) {
                            shooter.setPower(1);
                        }

                        if (llResult.getTy() >= -9) {
                            shooter.setPower(0.8);
                        }




                    }


                }
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("power", shooter.getPower());
            telemetry.update();
            }
        }
    }
}
