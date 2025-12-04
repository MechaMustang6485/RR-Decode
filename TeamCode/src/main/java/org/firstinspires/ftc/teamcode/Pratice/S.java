package org.firstinspires.ftc.teamcode.Pratice;





import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Config file
 */
@TeleOp(group="Primary")
public class S extends LinearOpMode{
    private Servo arm;
    private CRServo intake;
    private Servo stopper ;
    private int pos=96;
    private DcMotorEx revolver;
    private DcMotor shooter;
    private TouchSensor touch;
    private boolean touchVal=false;
    private boolean emptySlot;
    NormalizedColorSensor colorsensor;
    private boolean greeen;

    // if over shoot experiment with PIDposition to tune
    private double PIDposition=13;



    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            telemetry();
            getTouchSensor();
            getColor();
        }
        waitForStart();
        while (opModeIsActive()){
            teleOpControls();
            telemetry();
            getColor();
            getTouchSensor();
        }
    }

    public void initHardware() {
        initRevolver(PIDposition);
        //initShooter();
        initColorSens();
        //initIntake();
        //initArm();
        //initStopper();
        //initTouch();
        initColorSens();

    }
    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public void initColorSens(){
        colorsensor=hardwareMap.get(NormalizedColorSensor.class,"colorSensor");
        colorsensor.setGain(12);
    }

    public DetectedColor getColor(){
            // Read normalized color data
            NormalizedRGBA colors = colorsensor.getNormalizedColors();
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

            if (isGreen) {
                greeen=true;
            } else if (isPurple) {
                greeen=false;
            }


        telemetry.addData("Green",isGreen);
        telemetry.addData("Purple",isPurple);
        telemetry.addLine("a PPG");
        telemetry.addLine("b GPP");
        telemetry.addLine("x PGP");
        telemetry.update();
        return DetectedColor.UNKNOWN;
    }
    public void initIntake(){
        intake=hardwareMap.get(CRServo.class,"intake");
        intake.setPower(0);
    }

    public void initArm(){
        arm=hardwareMap.get(Servo.class,"arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(0.6);
    }
    public void initStopper(){
        stopper=hardwareMap.get(Servo.class,"stopper");
        stopper.setPosition(0.3);
    }

    public void initTouch(){
        touch=hardwareMap.get(TouchSensor.class,"touch");
    }
    public void getTouchSensor(){
        touchVal=touch.isPressed();
    }

    public void initRevolver(double PIDposition){
        revolver=hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPositionPIDFCoefficients(PIDposition);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setPower(0);
        revolver.setTargetPosition(0);
        revolver.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void initShooter(){
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void teleOpControls(){
        if(gamepad1.right_bumper){
            initStart();
        }
        if(gamepad1.a){
            initSequenceA();
        }
        if(gamepad1.b){
            initSequenceB();
        }
        if(gamepad1.x) {
            initSequenceC();
        }
        if(gamepad1.y){
            stopper.setPosition(-0.3);
            intake.setPower(-1);
            if(gamepad1.right_bumper){
                intake.setPower(0);
            }
            if(touch.isPressed()){
                revolver.setTargetPosition(96);
                sleep(2000);
                revolver.setTargetPosition(192);
                sleep(2000);
                revolver.setTargetPosition(288);
                sleep(2000);
                if(revolver.getTargetPosition()==288 && touch.isPressed()){
                    arm.setPosition(0);
                    intake.setPower(0);
                    telemetry.addLine("All slots filled");
                    telemetry.update();
                }
            }
        }

    }


    // revolver sequences
    // P P G
    public void initSequenceA(){
        initP();
        sleep(2000);
        shooter.setPower(0);
        initPP();
        sleep(2000);
        shooter.setPower(0);
        initPPG();
        sleep(3000);
        shooter.setPower(0);
    }
    public void initStart(){
        revolver.setPower(0.5);
        if(greeen){
            revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            revolver.setPower(0);
            revolver.setTargetPosition(0);
        }
    }
    //    P P G
    public void initP(){
        revolver.setTargetPosition(96);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shooter.setPower(1);
        revolver.setPower(0.5);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.update();
    }



    public void initPP(){
        // P P
        revolver.setTargetPosition(192);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("P");
        telemetry.update();

    }

    public void initPPG(){
        // P P G
        revolver.setTargetPosition(288);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("P");
        telemetry.addLine("G");
        telemetry.update();

    }

    // G P P
    public void initSequenceB(){
        initG();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initGP();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initGPP();
        sleep(3000);
        arm.setPosition(0.6);
        shooter.setPower(0);
    }

    public void initG(){
        // G
        revolver.setTargetPosition(0);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("G");
        telemetry.update();
    }

    public void initGP(){
        // G P
        revolver.setTargetPosition(96);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("G");
        telemetry.addLine("P");
        telemetry.update();
    }

    public void initGPP(){
        //G P P
        revolver.setTargetPosition(192);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("G");
        telemetry.addLine("P");
        telemetry.addLine("P");
        telemetry.update();
    }


    // P G P
    public void initSequenceC(){
        initP2();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initPG();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initPGP();
        sleep(3000);
        arm.setPosition(0.6);
        shooter.setPower(0);

    }

    public void initP2(){
        // P
        revolver.setTargetPosition(96);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.update();
    }

    public void initPG(){
        // P G
        revolver.setTargetPosition(288);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("G");
        telemetry.update();
    }

    public void initPGP(){
        // P G P
        revolver.setTargetPosition(192);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("G");
        telemetry.addLine("P");
        telemetry.update();
    }




    public void telemetry(){

        telemetry.update();
    }


}


