package org.firstinspires.ftc.teamcode.opmodemasterlist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp2021")
public class TeleOp2021 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx slides;
    private Servo tilt;




    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        tilt = hardwareMap.get(Servo.class, "tilt");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        acceptPosition();

        while (opModeIsActive()) {

            //all the buttons
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
            double r = Math.hypot(-1 * gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -1 * gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            //set drivetrain velocities
            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            intake.setPower(-1 * leftTrig1);




            if(dpadU2)
            {
                liftPosition();
                telemetry.addData("current", tilt.getPosition());
            }

            if(dpadD2)
            {
                deliverPosition();
                telemetry.addData("current", tilt.getPosition());
            }

            if(dpadR2){
                acceptPosition();
                telemetry.addData("current", tilt.getPosition());
            }

            telemetry.update();

            if (a1)
            {
                double temp = tilt.getPosition();
                tilt.setPosition((temp + .005));
                telemetry.addData("current", tilt.getPosition());
            }

            if (b1)
            {
                double temp = tilt.getPosition();
                tilt.setPosition(temp - .005);
                telemetry.addData("current", tilt.getPosition());
            }

            telemetry.update();







        }


    }

    private void acceptPosition() {
        tilt.setPosition(.95);
    }

    private void liftPosition() {
        tilt.setPosition(.89);
    }

    private void deliverPosition() {
        tilt.setPosition(.58);
    }

    private void deliver() {
        slides.setPower(.05);
        sleep(200);
        slides.setPower(0);
    }

    private void down() {
        slides.setPower(-.05);
        sleep(200);
        slides.setPower(0);
    }



}