package org.firstinspires.ftc.teamcode.Pratice;





import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Disabled
@Autonomous
public final class redAutoBackup extends LinearOpMode {
    private PIDFController controller;//important

    public static double p = 0.1, i = 0, d= 0;
    public static double f = 0.000001;

    public static int target = 57;
    public static int target2 = 249;
    public static int target3 = 155;

    private final double ticks_in_degree = 700/ 180.0;//changes depending on the motor

    private DcMotorEx Revolver;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap , new Pose2d(0,0,0));


        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .lineToXConstantHeading(30.3)
                        .strafeToLinearHeading(new Vector2d(30.3,28), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                        //.stopAndAdd(new Shooter())
                        .build());



    }

    public class Shooter {
        public Action spinUp() {
            controller.setPIDF(p, i, d, f);
            int revpose = Revolver.getCurrentPosition();
            double pid = controller.calculate(revpose, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;//math that sets the power
            Revolver.setPower(power);
            return null;
        }


    }
}

