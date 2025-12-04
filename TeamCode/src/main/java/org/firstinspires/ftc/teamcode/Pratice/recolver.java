package org.firstinspires.ftc.teamcode.Pratice;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class recolver extends LinearOpMode {

    public NormalizedColorSensor colorSensor;

    public DcMotorEx revolver;

    double purplee = 0;
    double greeen = 0;

    public void initHardware() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        colorSensor.setGain(12);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initHardware();

        while (opModeIsActive()) {
            getColor();
        }
    }

    public void getColor() {
        // Read normalized color data
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
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
                ((hue > 180 && hue < 240)) &&
                        val > 0.01;     // allow dark

        // Green: 80–160
        boolean isGreen =
                (hue > 120 && hue < 150) &&
                        val > 0.03;

        RevolSeq(isPurple, isGreen, hue, sat, val);
        telemetry.update();

    }

    public void RevolSeq(boolean isPurple, boolean isGreen, float hue, float sat, float val) {
            //revolver.setPower(0.2);

            telemetry.addData("Hue", hue);
            telemetry.addData("Saturation", sat);
            telemetry.addData("Value", val);

            //if (isGreen) {
            //    telemetry.addLine("G");
            //    greeen = greeen + 1;
            //    telemetry.update();
            //} else if (isPurple) {
            //    telemetry.addLine("P");
            //    purplee = purplee + 1;
            //    telemetry.update();
            //}

            //if(greeen + purplee >= 3) {
            //    revolver.setPower(0);
            //    telemetry.update();
            //}

        if (isGreen){
            revolver.setPower(1);
        } else if (isPurple) {
            revolver.setPower(0);
        }

    }
}
