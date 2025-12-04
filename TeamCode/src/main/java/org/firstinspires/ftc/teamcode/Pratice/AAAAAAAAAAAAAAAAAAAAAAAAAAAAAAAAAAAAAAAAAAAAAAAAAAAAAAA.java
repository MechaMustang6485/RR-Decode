package org.firstinspires.ftc.teamcode.Pratice;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(group = "Primary")
public class AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA extends LinearOpMode {

    private NormalizedColorSensor sensor;

    private DcMotorEx revolver;

    private Servo stopper;


    private TouchSensor touch;
    private boolean touchVal=false;

    @Override
    public void runOpMode() throws InterruptedException {

        sensor = hardwareMap.get(NormalizedColorSensor.class, "sensor");
        sensor.setGain(12);

        touch = hardwareMap.get(TouchSensor.class, "touch");
        if (touch.isPressed()) {
            touchVal = true;
        } else touchVal = false;

        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setDirection(Servo.Direction.FORWARD);
        stopper.setPosition(-0.3);


        revolver = hardwareMap.get(DcMotorEx.class, "revolver");



        waitForStart();

        while (opModeIsActive()) {
            getColor();
            telemetry.update();
        }
    }

    public void getColor() {
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

        // Send data to telemetry
        colorTelemetry(isPurple, isYellow, isGreen, isWhite, hue, sat, val);
        test(isPurple, isGreen);
    }

    public void test(boolean isPurple, boolean isGreen) {
       /* if (isGreen) {
            revolver.setPower(0);
            telemetry.addLine("G");
            sleep(500);
            revolver.setPower(0.3);
        } else if (isPurple) {
            revolver.setPower(0);
            telemetry.addLine("P");
            sleep(500);
            revolver.setPower(0.3);
        }

        if (gamepad1.yWasPressed()) {
            revolver.setPower(0.3);
        } else if (gamepad1.xWasPressed()) {
            revolver.setPower(0.3);

        } */

        if (touch.getValue() == 1 && isGreen || isPurple) {
            // Stop the revolver while touching
            revolver.setPower(0);

            // Check the color
            if (isGreen) {
                telemetry.addLine("G");
            } else if (isPurple) {
                telemetry.addLine("P");
            }

            if (gamepad1.xWasPressed()) {
                revolver.setPower(0.4);
            }
        }
        else {
            if (!isGreen && !isPurple) {
                revolver.setPower(0.3);
            } else {
                revolver.setPower(0);
            }
        }



    }

    public void colorTelemetry(boolean isPurple,
                               boolean isYellow,
                               boolean isGreen,
                               boolean isWhite,
                               float hue,
                               float sat,
                               float val) {

        telemetry.addData("Hue", hue);
        telemetry.addData("Saturation", sat);
        telemetry.addData("Value", val);
        telemetry.addData("TouchVal", touchVal);

        if (isPurple) {
            telemetry.addData("Color Detected", "Purple");
        } else if (isYellow) {
            telemetry.addData("Color Detected", "Yellow");
        } else if (isGreen) {
            telemetry.addData("Color Detected", "Green");
        } else if (isWhite) {
            telemetry.addData("Color Detected", "White");
        } else {
            telemetry.addData("Color Detected", "None");
        }
    }
}
