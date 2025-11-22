package org.firstinspires.ftc.teamcode.DecodeDrive;





import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class NutralRedNoBall extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(-70,-32), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                        .build());

    }

}

