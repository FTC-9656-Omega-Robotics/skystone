package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoBackend.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Amogh Blue Side")
        public class AmoghBlueSide extends LinearOpMode {
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
                phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

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
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

                // number of inches the robot strafes back depending
                // on the position of the skystone closest to the Skybridge
                double front = 0;
                double back = 0;
                double intakeAngle = 0;
                //All comments comment above what is being commented

                while (!isStopRequested() && !opModeIsActive()) {
                    xPosition = skyStoneDetector.foundRectangle().x;
                    yPosition = skyStoneDetector.foundRectangle().y;

                    if (xPosition >= 180 || xPosition < 40) { //TODO Tune these numbers
                        skystonePosition = "right";
                        back = 14;
                    } else if (xPosition > 130) {//x = 12
                        skystonePosition = "center";
                        back = 8;
                    } else {
                        skystonePosition = "left";
                        back = 2.5;
                    }

                    telemetry.addData("xPos", xPosition);
                    telemetry.addData("yPos", yPosition);
                    telemetry.addData("SkyStone Pos", skystonePosition);
                    telemetry.update();
                }
                int x = 0;
                waitForStart();

                //Initialization for teleop/side gripper
                robot.cap.setPosition(0.98);
                robot.centerGripper.setPosition(0.55);
                //gripper to super open?
                robot.rightGripper.setPosition(0.03);
                //elbow down
                robot.arm.setTargetPosition(-200);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(.5);
                //teleop arm up


                //gets in position and strafes to skystone
                motionMethods.moveMotionProfile(back, 1);
                //change the distance to back once tuned
                motionMethods.strafe(180, .85, 1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                sleep(100);
                //double time = runtime.milliseconds();
                //while(runtime.milliseconds()<time+700);
                //intakes stone
                robot.rightGripper.setPosition(0.03);
                //elbow down
                sleep(400);
                robot.cap.setPosition(.40);
                //gripper closed
                sleep(750);
                robot.rightGripper.setPosition(.31);
                //elbow up
                sleep(500);

                //moving to the other side
                motionMethods.strafe(0, .45, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.moveMotionProfile(59.5 - back,1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                //go across the field to deposit in foundation
                motionMethods.strafe(180, .4, 0.5);
                //time for low voltage is .35

                //dump on foundation
                robot.rightGripper.setPosition(0.02);
                sleep(500);
                //elbow down
                robot.cap.setPosition(.98);
                //gripper to super open?
                robot.rightGripper.setPosition(.31);
                //elbow up

                //move to position to intake 2nd skystone
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.strafe(0, .36, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                robot.drivetrain.reverseDirection();
                motionMethods.moveMotionProfileReverse(74.8 - back,1);
                //subtract one or add one depending on voltage 81.3 and 82.3
                //change distance for 2nd skystone
                robot.drivetrain.reverseDirection();
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );

                //final line up/intake of 2nd skystone
                robot.rightGripper.setPosition(0.03);
                sleep(400);
                motionMethods.strafe(180, .35, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                robot.cap.setPosition(.40);
                //gripper closed
                sleep(800);
                robot.rightGripper.setPosition(.31);
                //elbow up
                sleep(800);

                //move to deposit on foundation
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.strafe(0, .45, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.moveMotionProfileZeroDegrees(95.5 - back ,1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                //change distance experimentally

                //strafes and deposits stone
                motionMethods.strafe(180, .30, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1);
                sleep(100);
                robot.rightGripper.setPosition(0.03);
                //elbow down
                sleep(400);
                robot.cap.setPosition(.98);
                //gripper open
                sleep(500);
                robot.rightGripper.setPosition(.31);
                //elbow up
                sleep(200);
                robot.centerGripper.setPosition(.85);
                sleep(100);
                robot.drivetrain.reverseDirection();
                motionMethods.moveMotionProfileReverse(7.5,1);
                robot.drivetrain.reverseDirection();
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1);


                //foundation pull
                motionMethods.turnUsingPIDVoltage(-90,1);

                robot.drivetrain.reverseDirection();
                motionMethods.moveMotionProfileReverse(8,1);
                robot.drivetrain.reverseDirection();
                robot.centerGripper.setPosition(1);
                sleep(200);
                motionMethods.moveMotionProfile(24,1);
                motionMethods.turnUsingPIDVoltageFieldCentric(-180,1);
                sleep(300);
                robot.centerGripper.setPosition(.55);
                sleep(700);


                //final line up for park


                motionMethods.turnTheCoolVivaWayFieldCentric(-270,1);
                motionMethods.strafeLeft(25,1);





            }
}

