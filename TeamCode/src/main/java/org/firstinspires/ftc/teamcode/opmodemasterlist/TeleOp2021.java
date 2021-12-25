package org.firstinspires.ftc.teamcode.opmodemasterlist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp2021")
public class TeleOp2021 extends LinearOpMode {

    //Declaring Motor Classes
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx slides;
    private Servo tilt;




    public void runOpMode() throws InterruptedException {

        //Initializing the Motor Classes
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        tilt = hardwareMap.get(Servo.class, "tilt");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        acceptPosition();

        waitForStart();




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
            double r = Math.hypot(-1 * gamepad1.left_stick_x, -1 * gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, -1 * gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -1 * gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + .5 * rightX;
            final double v2 = r * Math.sin(robotAngle) - .5 * rightX;
            final double v3 = r * Math.sin(robotAngle) + .5 * rightX;
            final double v4 = r * Math.cos(robotAngle) - .5 * rightX;

            //set drivetrain velocities
            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            intake.setPower(-1 * leftTrig1);





            if(dpadU2) { topLevel(); }

            if(dpadR2) { midLevel(); }

            if(dpadD2) { lowLevel(); }

            if(dpadL2) { ground(); slides.setPower(0);}

            if(rightBump2) {slides.setPower(0); }


            if(y2) { acceptPosition(); }

            if(b2) { liftPosition(); }

            if(a2) { deliverPosition(); }


            telemetry.addData("current encoder value", slides.getCurrentPosition());
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

    private void topLevel() {

        slides.setTargetPosition(367);
        slides.setPower(.2);
  /*      while (slides.getCurrentPosition() < slides.getTargetPosition())
        {
            idle();
        }
       */
    }

    private void midLevel() {

        slides.setTargetPosition(216);
        slides.setPower(.2);
     /*   while (opModeIsActive() && slides.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }
        */


    }

    private void lowLevel() {

        slides.setTargetPosition(76);
        slides.setPower(.2);
  /*      while (opModeIsActive() && slides.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
       }
   */
    }

    private void ground() {

        slides.setTargetPosition(0);
        slides.setPower(.2);
   /*     while (opModeIsActive() && slides.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }
        */


    }






}

//            telemetry.update();

            /*
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

*/
