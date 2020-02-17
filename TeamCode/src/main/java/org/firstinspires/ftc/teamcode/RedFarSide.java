package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoBackend.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Red Far Side")
public class RedFarSide extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double robotSpeed = 0.45;
    OmegaBot robot;
    OmegaPID drivePID;
    MotionMethods motionMethods;
    private OpenCvCamera phoneCam;
    private CustomSkystoneDetector skyStoneDetector;

    String skystonePosition = "none";
    double xPosition;
    double yPosition;

    public void runOpMode() {
        robot = new OmegaBot(telemetry, hardwareMap);
        drivePID = robot.drivePID;
        motionMethods = new MotionMethods(robot, telemetry, this);
        /*
         */
        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new CustomSkystoneDetector();
        skyStoneDetector.useDefaults();
        phoneCam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        // number of inches the robot strafes back depending
        // on the position of the skystone closest to the Skybridge
        double back = 0;
        //All comments comment above what is being commented

        waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            xPosition = skyStoneDetector.foundRectangle().x;
            yPosition = skyStoneDetector.foundRectangle().y;

            if (xPosition >= 60) { //TODO Tune these numbers
                skystonePosition = "right";
                back = 3;
            } else if (xPosition > 2) {//x = 12
                skystonePosition = "center";
                back = 7.5;
            } else {
                skystonePosition = "left";
                back = 12;
            }

            telemetry.addData("xPos", xPosition);
            telemetry.addData("yPos", yPosition);
            telemetry.addData("SkyStone Pos", skystonePosition);
            telemetry.update();
        }

        // set initial values for arm, block gripper, pivot, and intakes
        robot.arm.setTargetPosition(-400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        robot.blockGripper.setPosition(.75);
        robot.pivot.setPosition(.62);
        sleep(500);
        robot.leftIntake.setPower(-1);
        robot.rightIntake.setPower(1);

        // strafe backward and into the row of stones
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(back,1);
        robot.drivetrain.reverseDirection();
        motionMethods.strafe(180, 1, .75);//if voltage is around 12, 1.1, if voltage is higher than that, 1
        motionMethods.moveMotionProfile(7,1);
        sleep(500);

        // intake the skystone closest to the skybridge
        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        robot.blockGripper.setPosition(0.2);
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);
        sleep(500);
        if(skystonePosition.equals("left")) back += 3;
        motionMethods.moveMotionProfile(back, 1);

        // turns left to face red side
        motionMethods.turnUsingPIDVoltageFieldCentric(90,.5);

        // move back toward blue wall
        robot.arm.setTargetPosition(-110);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(10, 1);
        robot.drivetrain.reverseDirection();

        // turn right to return to original angle (forward would be parallel to wall)
        motionMethods.turnUsingPIDVoltageFieldCentric(0, .5);

       // moves towards foundation/building zone
        motionMethods.moveMotionProfile(56, 1);

        //turn right to face blue wall (robot centric)
        motionMethods.turnUsingPIDVoltageFieldCentric(-90, .5);

        //back up to retrieve the waffle pan
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(8, 1);
        robot.drivetrain.reverseDirection();
        //grippers grip the waffle pan

        robot.centerGripper.setPosition(1.00);
        //arm moves down and drops the brick
        robot.arm.setTargetPosition(-1700);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        sleep(500);
        //let go of brick
        robot.blockGripper.setPosition(.75);
        sleep(750);
        robot.arm.setTargetPosition(-200);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        motionMethods.moveMotionProfile(39,1);
        //we wait a few seconds before moving the arm up
        sleep(500);
        robot.centerGripper.setPosition(.51);
        //turns to face quarry

        motionMethods.turnUsingPIDVoltageFieldCentric(-90, .5);
        //corrects for position

        //strafes right
        motionMethods.strafe(0, .8, 1);

        //corrects position before reversing
        motionMethods.turnUsingPIDVoltageFieldCentric(-90, .5);

        //move to far side of under bridge
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(18, 1);
        robot.drivetrain.reverseDirection();
        //turn to face blocks
        motionMethods.turnUsingPIDVoltageFieldCentric(-180,.5);
        //move under bridge
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(8,1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(-180,.5);
        motionMethods.moveMotionProfile(18, 1);

        runtime.reset();
    }
}