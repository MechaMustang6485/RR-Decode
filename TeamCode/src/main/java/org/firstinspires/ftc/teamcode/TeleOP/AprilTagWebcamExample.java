package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous
public class AprilTagWebcamExample extends OpMode {
    OpenCV apriltagwebcam = new OpenCV();
    public DcMotor m1;

    @Override
    public void init() {
        apriltagwebcam.init(hardwareMap, telemetry);
        m1 = hardwareMap.get(DcMotor.class, "m1");

    }

    @Override
    public void loop() {
        //update vision
        apriltagwebcam.update();
        AprilTagDetection id24 = apriltagwebcam.getTagBySpecificId(24);
        AprilTagDetection id20 = apriltagwebcam.getTagBySpecificId(20);
        apriltagwebcam.displayDetectionTelemetry(id24);
        if (id24 != null) {
            m1.setPower(1);
        } else {
            m1.setPower(0);
        }

        if (id20 != null) {
            m1.setPower(-1);
        } else {
            m1.setPower(0);
        }

    }
}
