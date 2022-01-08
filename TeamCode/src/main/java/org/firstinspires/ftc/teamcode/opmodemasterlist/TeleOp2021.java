package org.firstinspires.ftc.teamcode.opmodemasterlist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp2021")
public class TeleOp2021 extends LinearOpMode {

    //Declaring motor classes
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx slides;
    private DcMotorEx carousel;
    private Servo tilt;
    private Servo arm;
    private ColorSensor color;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    private int[] yellow = {100, 100, 100}; // R G B
    private int[] white = {100, 100, 100}; // R G B
    private HashMap<String, int[]> colors = new HashMap() {{
        put("yellow", yellow);
        put("white", white);
    }};

    private int colorCloseness(int[] rgb) {
        return (
          Math.abs(color.red() - rgb[0]) +
          Math.abs(color.green() - rgb[1]) +
          Math.abs(color.blue() - rgb[2])
        ) / 3;
    }

    private String closestColor() {
        // Outputs the closest color
        // If none are close enough, it returns "none"
        int range = 25; // How much the closeness can be for it to choose the color
        String lowestKey = "none";
        int lowestCloseness = 2147483647;
        for (Map.Entry<String, int[]> rgb : colors.entrySet()) {
            int closeness = colorCloseness(rgb.getValue());
            if (closeness < range && closeness < lowestCloseness) {
                lowestKey = rgb.getKey();
                lowestCloseness = closeness;
            }
        }
        return lowestKey;
    }

    private void setLED(String color) {
        if (color == "red") {
          greenLED.setState(false);
          redLED.setState(true);
        } else if (color == "green") {
            greenLED.setState(true);
            redLED.setState(false);
        } else if (color == "amber") {
            greenLED.setState(true);
            redLED.setState(true);
        } else {
            greenLED.setState(false);
            redLED.setState(false);
        }
    }

    public void runOpMode() throws InterruptedException {

        //Initializing the Motor Classes
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        tilt = hardwareMap.get(Servo.class, "tilt");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        arm = hardwareMap.get(Servo.class, "arm");
        color = hardwareMap.get(ColorSensor.class, "color");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        acceptPosition();

        waitForStart();

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        while (opModeIsActive()) {
            //initialise all the buttons
            boolean dpadU1 = gamepad1.dpad_up;
            boolean dpadD1 = gamepad1.dpad_down;
            boolean dpadL1 = gamepad1.dpad_left;
            boolean dpadR1 = gamepad1.dpad_right;

            boolean leftBump1 = gamepad1.left_bumper;
            boolean rightBump1 = gamepad1.right_bumper;

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;

            double leftTrig1 = gamepad1.left_trigger;
            double rightTrig1 = gamepad1.right_trigger;


            boolean dpadU2 = gamepad2.dpad_up;
            boolean dpadD2 = gamepad2.dpad_down;
            boolean dpadL2 = gamepad2.dpad_left;
            boolean dpadR2 = gamepad2.dpad_right;

            boolean leftBump2 = gamepad2.left_bumper;
            boolean rightBump2 = gamepad2.right_bumper;

            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double leftTrig2 = gamepad2.left_trigger;
            double rightTrig2 = gamepad2.right_trigger;


            //annoying trig drivetrain stuff
            double r = Math.hypot(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            //set drivetrain velocities
            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);


            //intake
            if (leftTrig1 > rightTrig1)
                intake.setPower(-1 * leftTrig1);
            else
                intake.setPower(rightTrig1);


            //carousel
            if (x2)
                carousel.setPower(-.4);
            else if (rightTrig2 > leftTrig2)
                carousel.setPower(0.25 * rightTrig2);
            else
                carousel.setPower(-.25 * leftTrig2);


            //inching each direction
            if (dpadU1) {
                frontLeft.setPower(.2);
                frontRight.setPower(.2);
                backLeft.setPower(.2);
                backRight.setPower(.2);
                sleep(50);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            if(dpadD1)
            {
                frontLeft.setPower(-.2);
                frontRight.setPower(-.2);
                backLeft.setPower(-.2);
                backRight.setPower(-.2);
                sleep(50);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            if(dpadR1)
            {
                frontLeft.setPower(.2);
                frontRight.setPower(-.2);
                backLeft.setPower(-.2);
                backRight.setPower(.2);
                sleep(50);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            if(dpadL1)
            {
                frontLeft.setPower(-.2);
                frontRight.setPower(.2);
                backLeft.setPower(.2);
                backRight.setPower(-.2);
                sleep(50);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            if(leftBump1)
            {
                frontLeft.setPower(-.2);
                frontRight.setPower(.2);
                backLeft.setPower(-.2);
                backRight.setPower(.2);
                sleep(50);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            if(rightBump1)
            {
                frontLeft.setPower(.2);
                frontRight.setPower(-.2);
                backLeft.setPower(.2);
                backRight.setPower(-.2);
                sleep(50);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }



            //slides and servo delivery and return
            if(dpadU2) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                tiltPosition(); //maybe
                topLevel();
                while (slides.getCurrentPosition() < slides.getTargetPosition()) {}
                deliverPosition();
                frontLeft.setPower(v1);
                frontRight.setPower(v2);
                backLeft.setPower(v3);
                backRight.setPower(v4);
            }

            if(dpadR2) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                tiltPosition();
                midLevel();
                while (slides.getCurrentPosition() < slides.getTargetPosition()) {}
                deliverPosition();
                frontLeft.setPower(v1);
                frontRight.setPower(v2);
                backLeft.setPower(v3);
                backRight.setPower(v4);
            }

            if(dpadD2) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                tiltPosition();
                lowLevel();
                while (slides.getCurrentPosition() < slides.getTargetPosition()) {}
                deliverPosition();
                frontLeft.setPower(v1);
                frontRight.setPower(v2);
                backLeft.setPower(v3);
                backRight.setPower(v4);
            }

            if(leftBump2) {

                acceptPosition();
                sleep(300);
                ground();

            }



            //emergency stop
            if(dpadL2) { slides.setPower(0); }

            //emergency servo adjustment
            if(y2) { acceptPosition(); }
            if(a2) { deliverPosition(); }

            //eject
            if(b2) { tilt.setPosition(.86); }

            //arm adjustment
            //while(leftTrig2 > 0) { arm.setPosition(arm.getPosition() + 0.1 * leftTrig2); }
            //while(rightTrig2 > 0) { arm.setPosition(arm.getPosition() - 0.1 * rightTrig2); }

            String currentColor = closestColor();
            switch (currentColor) {
                case "yellow":
                    setLED("red");
                case "white":
                    setLED("green");
                default:
                    setLED("");
            }

            telemetry.addData("Color", currentColor);
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }


    }

    //transfer tilt methods
    private void acceptPosition() {
        tilt.setPosition(.92);
    }
    private void tiltPosition() { tilt.setPosition(.89);}
    private void deliverPosition() { tilt.setPosition(.48); }

    //arm methods
    private void pickUp() { arm.setPosition(.5); }
    private void deliverCup() { arm.setPosition(.3); }
    private void restArm() { arm.setPosition(0); }


    //slide methods
    private void topLevel() {

        slides.setTargetPosition(367);
        slides.setPower(1);

    }

    private void midLevel() {

        slides.setTargetPosition(225);
        slides.setPower(1);

    }

    private void lowLevel() {

        slides.setTargetPosition(76);
        slides.setPower(1);

    }

    private void ground() {

        slides.setTargetPosition(0);
        slides.setPower(1);

    }


}

