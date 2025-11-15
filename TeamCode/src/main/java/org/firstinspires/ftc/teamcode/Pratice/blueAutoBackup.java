package org.firstinspires.ftc.teamcode.DecodeDrive;





import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Disabled
@Autonomous
public final class blueAutoBackup extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap , new Pose2d(0,0,0));


        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .lineToXConstantHeading(30.3)
                        .strafeToLinearHeading(new Vector2d(30,-28), Math.toRadians(0), (pose2dDual, posePath, v) -> 40)
                        .build());

    }





}

