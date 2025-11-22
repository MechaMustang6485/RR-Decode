package org.firstinspires.ftc.teamcode.DecodeDrive;





import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class NutralBlueNoBall extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(-70,32), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                        .build());

    }

}

