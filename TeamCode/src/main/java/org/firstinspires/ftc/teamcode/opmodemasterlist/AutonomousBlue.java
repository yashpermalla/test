package org.firstinspires.ftc.teamcode.opmodemasterlist;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

import static java.lang.Math.toRadians;

@Autonomous



public class AutonomousBlue extends LinearOpMode {


    protected DcMotorEx frontLeft;
    protected DcMotorEx frontRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx slides;
    private DcMotorEx carousel;
    private Servo tilt;
    OpenCvWebcam webcam;
    BNO055IMU imu;
    Orientation angles;
    protected double globalAngle;
    SamplePipeline pipeline;

    private ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SamplePipeline();
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        tilt = hardwareMap.get(Servo.class, "tilt");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        imu = hardwareMap.get(BNO055IMU.class, "imu");



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        webcam.setPipeline(pipeline);


        acceptPosition();

        Pose2d myPose = new Pose2d(-35, 62, toRadians(270));


        drive.setPoseEstimate(myPose);

        Trajectory myTrajectory1 = drive.trajectoryBuilder(myPose, false)
                .forward(5)
                .build();



        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .forward(19,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)


                )
                .build();

        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end(), false)
                .back(47)
                .build();

        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory3.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .back(12)
                .build();

        Trajectory myTrajectory5 = drive.trajectoryBuilder(myTrajectory4.end(), false)
                .forward(14)
                .build();


        Trajectory myTrajectory5a = drive.trajectoryBuilder(myTrajectory5.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .strafeLeft(11)
                .build();

        Trajectory myTrajectory6 = drive.trajectoryBuilder(myTrajectory5a.end(), false)
                .forward(55)
                .build();

        Trajectory myTrajectory7 = drive.trajectoryBuilder(myTrajectory6.end(), false)
                .back(55)
                .build();

        Trajectory myTrajectory8 = drive.trajectoryBuilder(myTrajectory7.end(), false)
                .strafeRight(10)
                .build();

        Trajectory myTrajectory9 = drive.trajectoryBuilder(myTrajectory8.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .back(14)
                .build();

        resetAngle();

        webcam.setMillisecondsPermissionTimeout(1000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("error");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("position", pipeline.position);
        telemetry.update();

        String location = pipeline.position;


        drive.followTrajectory(myTrajectory1);

        drive.turn(Math.toRadians(-90));
        //rotate(-90);

        drive.followTrajectory(myTrajectory2);
        timer.reset();
        while(timer.time(TimeUnit.SECONDS) < 3) {
            frontLeft.setPower(.08);
            frontRight.setPower(.08);
            backLeft.setPower(.08);
            backRight.setPower(.08);
            carousel.setPower(0.3);

        }
        carousel.setPower(0);
        //use motor here

        drive.followTrajectory(myTrajectory3);

        drive.turn(Math.toRadians(-90));
        //rotate(90);

        drive.followTrajectory(myTrajectory4);



        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        tiltPosition(); //maybe

        if(location == "LEFT"){
            lowLevel();
        }else if(location == "MIDDLE"){
            midLevel();
        }else {
            topLevel();
        }

        while (opModeIsActive() && slides.getCurrentPosition() < slides.getTargetPosition()-3) {}
        deliverPosition();

        sleep(2000);

        acceptPosition();
        sleep(300);
        ground();





        drive.followTrajectory(myTrajectory5);

        drive.turn(Math.toRadians(-90));
        //rotate(90);

        drive.followTrajectory(myTrajectory5a);
        drive.followTrajectory(myTrajectory6);

        timer.reset();
        while(timer.time(TimeUnit.SECONDS) < 1) {
            frontLeft.setPower(.1);
            frontRight.setPower(.1);
            backLeft.setPower(.1);
            backRight.setPower(.1);
            intake.setPower(-1);

        }

        intake.setPower(0);

        //pick up blocks

//       drive.followTrajectory(myTrajectory7);
//
//
//       //rotate(-90);
//
//      drive.followTrajectory(myTrajectory8);
//
//        drive.turn(Math.toRadians(90));
//
//      drive.followTrajectory(myTrajectory9);
//
//
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//        tiltPosition(); //maybe
//        topLevel();
//        while (slides.getCurrentPosition() < slides.getTargetPosition()) {}
//        deliverPosition();
//
//        sleep(1000);
//
//        acceptPosition();
//        sleep(300);
//        ground();
//
//



//        //drop blocks here
//
//
//        drive.followTrajectory(myTrajectory5);
//
//        drive.turn(Math.toRadians(90));
//        //rotate(90);
//
//        drive.followTrajectory(myTrajectory5a);
//        drive.followTrajectory(myTrajectory6);



    }
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        private volatile String position = "";
        private volatile double avg = 0;
        private volatile int avg1 = 0, avg2 = 0, avg3 = 0;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        //Base Mat which stores RGB pixel values
        Mat mat = new Mat();

        //Mat that store black and white img with red being white
        Mat thresh = new Mat();

        //Mat with edges being detected
        Mat edges = new Mat();

        Mat subedgesone = new Mat(), subedgestwo = new Mat(), subedgesthree = new Mat();




        @Override
        public Mat processFrame(Mat input) {

            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            // Make a working copy of the input matrix in HSV
            Scalar lowHSV = new Scalar(new double[]{0,130,0});  // lower bound HSV for RED
            Scalar highHSV = new Scalar(new double[]{180,225,191}); // higher bound HSV for RED

            //Translates imput to detect the shade of red we want
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                position = "";
                return input;
            }


            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(mat, lowHSV, highHSV, thresh);

            //Detects Edges
            Imgproc.Canny(thresh, edges, 60, 60 * 2);

            Size a = edges.size();
            double rows = a.height;
            double columns = a.width;

            //Creating 3  rectangles to set boundaries of whether the cup in inside one of these rectangles.
            subedgesone = edges.submat(new Rect(new Point(0,70), new Point(100, 160)));
            subedgestwo = edges.submat(new Rect(new Point(100,70), new Point(220, 160)));
            subedgesthree = edges.submat(new Rect(new Point(220,70), new Point(320, 160)));

            avg1 = (int) Core.mean(subedgesone).val[0];
            avg2 = (int) Core.mean(subedgestwo).val[0];
            avg3 = (int) Core.mean(subedgesthree).val[0];

            if (avg1 > 2)
                position = "LEFT";
            else if (avg2 > 2)
                position = "MIDDLE";
            else if (avg3 > 2)
                position = "RIGHT";


            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return edges;
        }


        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {

                webcam.resumeViewport();
            }
        }


    }


    public void resetAngle()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation newangles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = newangles.firstAngle - angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        angles = newangles;

        return globalAngle;
    }



    public void rotate(double degrees) {

        if (degrees < -20) {

            frontLeft.setPower(+.35);
            backLeft.setPower(+.35);
            frontRight.setPower(-.35);
            backRight.setPower(-.35);

            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees + 20) {
                telemetry.addData("Current Angle", getAngle());

            }
            frontLeft.setPower(+.1);
            backLeft.setPower(+.1);
            frontRight.setPower(-.1);
            backRight.setPower(-.1);
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("Current Angle", getAngle());

            }
        }
        else if (degrees < 0) {

            frontLeft.setPower(+.1);
            backLeft.setPower(+.1);
            frontRight.setPower(-.1);
            backRight.setPower(-.1);

            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("Current Angle", getAngle());

            }
        }
        else if (degrees > 20) {

            frontLeft.setPower(-.35);
            backLeft.setPower(-.35);
            frontRight.setPower(+.35);
            backRight.setPower(+.35);

            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() < degrees - 20) {
                telemetry.addData("Current Angle", getAngle());

            }
            frontLeft.setPower(-.1);
            backLeft.setPower(-.1);
            frontRight.setPower(+.1);
            backRight.setPower(+.1);
            while (opModeIsActive() && getAngle() < degrees ) {
                telemetry.addData("Current Angle", getAngle());

            }

        }
        else if (degrees > 0)
        {
            frontLeft.setPower(-.1);
            backLeft.setPower(-.1);
            frontRight.setPower(+.1);
            backRight.setPower(+.1);
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() < degrees ) {
                telemetry.addData("Current Angle", getAngle());

            }
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        resetAngle();
        telemetry.addData("Current Angle", getAngle());


    }


    //servo methods
    private void acceptPosition() { tilt.setPosition(.92); }
    private void tiltPosition() { tilt.setPosition(.89);}
    private void deliverPosition() { tilt.setPosition(.48); }


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
