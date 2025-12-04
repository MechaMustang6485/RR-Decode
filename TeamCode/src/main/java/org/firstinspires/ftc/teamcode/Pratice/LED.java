package org.firstinspires.ftc.teamcode.Pratice;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LED extends LinearOpMode {

    private boolean LEDtimer = false;
    private int LEDdelay = 2000;
    RevBlinkinLedDriver lights;

    public void initLED() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void resetLED() {
        LEDtimer = true;
        resetRuntime();
    }

    public void updateLED() {
        if(LEDtimer && time < 1){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
        }
        if (LEDtimer && time < 120) {//2:10
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }
    }

    @Override
    public void runOpMode() {
        initLED();
        while(!isStarted()) {}
        waitForStart();
        resetLED();
        while (opModeIsActive()) {
            updateLED();
        }
    }
}

