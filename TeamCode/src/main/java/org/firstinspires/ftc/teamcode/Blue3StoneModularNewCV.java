package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import kotlin.Unit;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "Blue 3 Stone Modular New CV")
public class Blue3StoneModularNewCV extends LinearOpMode {
    // ----------------- HARDWARE --------------------------------
    OmegaBotRR robot;
    SampleMecanumDriveBase drive;


    // ----------------- COMPUTER VISION -------------------------
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = -0.5f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;


    // ----------------- CONSTANT POSITIONS ----------------------

    // initial position of the robot
    final int INIT_X = -35;
    final int INIT_Y = 67;
    final Pose2d ROBOT_INIT_POSITION = new Pose2d(INIT_X, INIT_Y, Math.toRadians(180)); // 180 deg because faces wall with stones

    // skystone coordinates
    // 1 is closest to bridge, 6 is closest to wall
    // NOTE: these coordinates should only be used in pairs (1-4, 2-5, 3-6) when running full auto paths
    // they may not actually represent where the skystones are; they're just coordinates that work
    // when tuning each of the 3 blue autonomous paths
    final int SKYSTONE_1_X = -17;
    final int SKYSTONE_2_X = -25;
    final int SKYSTONE_3_X = -36;
    final int SKYSTONE_4_X = -43;
    final int SKYSTONE_5_X = -52;
    final int SKYSTONE_6_X = -62;

    final int SKYSTONE_1_Y = 31;
    final int SKYSTONE_2_Y = 32;
    final int SKYSTONE_3_Y = 32;
    final int SKYSTONE_4_Y = 36;
    final int SKYSTONE_5_Y = 36;
    final int SKYSTONE_6_Y = 36;

    // a bit of space between robot and neutral bridge
    final int UNDER_BLUE_BRIDGE_X = 0;
    final int UNDER_BLUE_BRIDGE_Y = 40;

    // parked position coordinates
    final int PARKED_X = 0;
    final int PARKED_Y = 33;

    // after gripping foundation, splines to this position to move foundation into building site
    final int FOUNDATION_END_X = 30;
    final int FOUNDATION_END_Y = 50;
    final Pose2d FOUNDATION_END_POS = new Pose2d(FOUNDATION_END_X, FOUNDATION_END_Y, Math.toRadians(180));

    // dump position coordinates
    // far = close to wall, close = close to bridge
    final int DUMP_FAR_X = 64;
    final int DUMP_MID_X = 53;
    final int DUMP_CLOSE_X = 48;

    final int DUMP_FAR_Y = 33;
    final int DUMP_MID_Y = 31;
    final int DUMP_CLOSE_Y = 33;

    // ----------------- DYNAMIC POSITIONS --------------------------

    // coordinates for skystone closest to wall
    int skystoneWallX;
    int skystoneWallY;

    // coordinates for skystone closest to bridge
    int skystoneBridgeX;
    int skystoneBridgeY;

    // coordinates for regular stone closest to bridge
    int stoneX;
    int stoneY;


    @Override
    public void runOpMode() throws InterruptedException {
        // ----------------- INITIALIZATION --------------------

        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        drive = new SampleMecanumDriveREV(hardwareMap);

        // ----------------- COMPUTER VISION --------------------

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
        //width, height


        // ------------ CHOOSING AN AUTO PATH --------------

        // use CV to detect location of the skystone
        while (!isStopRequested() && !opModeIsActive()) {

            // display position values
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);


            boolean skystoneAtPos1 = valLeft == 0;
            boolean skystoneAtPos2 = valMid == 0;

            if (skystoneAtPos1) {
                skystoneWallX = SKYSTONE_4_X;
                skystoneWallY = SKYSTONE_4_Y;

                skystoneBridgeX = SKYSTONE_1_X;
                skystoneBridgeY = SKYSTONE_1_Y;

                // nearest regular stone is at pos 2
                // stoneX = [?];
                // stoneY = [?];
            } else if (skystoneAtPos2) {
                skystoneWallX = SKYSTONE_5_X;
                skystoneWallY = SKYSTONE_5_Y;

                skystoneBridgeX = SKYSTONE_2_X;
                skystoneBridgeY = SKYSTONE_2_Y;

                // nearest regular stone is at pos 1
                // stoneX = [?];
                // stoneY = [?];
            } else {
                skystoneWallX = SKYSTONE_6_X;
                skystoneWallY = SKYSTONE_6_Y;

                skystoneBridgeX = SKYSTONE_3_X;
                skystoneBridgeY = SKYSTONE_3_Y;

                // nearest regular stone is at pos 1
                // stoneX = [?];
                // stoneY = [?];
            }
        }

        // 3-stone is still in development
        stoneX = 0; // null
        stoneY = 0; // null

        waitForStart();

        if (isStopRequested()) return;

        // --------- EXECUTE CHOSEN AUTO PATH ----------

        executeAutoPath(skystoneBridgeX, skystoneBridgeY, skystoneWallX, skystoneWallY, stoneX, stoneY);
    }

    /**
     * Executes a 2-stone autonomous path that completes the following
     * tasks on the blue side of the field:
     *
     * - Delivers 2 skystones under the alliance skybridge
     * - Dumps 2 skystones onto the foundation
     * - Moves the foundation to the building zone
     * - Parks under the alliance skybridge
     *
     * In development: 3-stone
     *
     * @param skystoneBridgeX  x coordinate of skystone closest to the bridge
     * @param skystoneBridgeY  y coordinate of skystone closest to the bridge
     * @param skystoneWallX    x coordinate of skystone closest to the wall
     * @param skystoneWallY    y coordinate of skystone closest to the wall
     * @param stoneX           x coordinate of regular stone closest to the bridge
     * @param stoneY           y coordinate of regular stone closest to the bridge
     */
    public void executeAutoPath(int skystoneBridgeX, int skystoneBridgeY, int skystoneWallX, int skystoneWallY, int stoneX, int stoneY) {
        // heading of robot (in deg) when it first moves under the blue bridge
        final int BRIDGE_ANGLE = 175;

        // before moving, get side front elbow and gripper ready
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_READY);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_READY);

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // move to first skystone (closest to wall)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(skystoneWallX, skystoneWallY)) // strafe to first skystone
                        .build()
        );

        // pick up first skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(700);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(300);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // move away from stones a bit so that the gripped stone doesn't hit the other ones when robot moves
                        .strafeTo(new Vector2d(skystoneWallX, skystoneWallY + 3))
                        .reverse()
                        .splineTo(new Pose2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y, Math.toRadians(BRIDGE_ANGLE))) // spline to under blue bridge
                        .splineTo(new Pose2d(DUMP_FAR_X, DUMP_FAR_Y, Math.toRadians(180))) // spline to farthest dumping position
                        .strafeTo(new Vector2d(DUMP_FAR_X, DUMP_FAR_Y  - 2)) // strafe closer to foundation to dump
                        .build()
        );

        // dump first skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(300);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_READY);
        sleep(300);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(300);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_STOWED);
        sleep(300);

        // move to second skystone (closest to bridge)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y, Math.toRadians(180))) // spline to under blue bridge
                        .addMarker( () -> {
                            // set side front elbow and gripper to ready position for efficiency
                            robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_READY);
                            robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_READY);

                            return Unit.INSTANCE;
                        })
                        .strafeTo(new Vector2d(skystoneBridgeX, skystoneBridgeY)) // strafe to second skystone
                        .build()
        );

        // pick up second skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(700);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(300);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse direction to go back to foundation
                        // move away from stones a bit so that the gripped stone doesn't hit the other ones when robot moves
                        .strafeTo(new Vector2d(skystoneBridgeX, skystoneBridgeY + 3))
                        .strafeTo(new Vector2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y - 4)) // strafe to under blue bridge (Y coordinate edited for alliance partner safety)
                        .splineTo(new Pose2d(DUMP_MID_X, DUMP_MID_Y, Math.toRadians(180))) // spline to middle dumping position
                        .strafeTo(new Vector2d(DUMP_MID_X, DUMP_MID_Y - 6)) // strafe closer to foundation
                        .build()
        );


        // dump second skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(300);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_READY);
        sleep(300);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(300);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_STOWED);

        /*

        // move to third stone (a regular one)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse direction to go back to quarry
                        .splineTo(UNDER_BLUE_BRIDGE_POS) // spline to under blue bridge
                        .addMarker( () -> { // move side front elbow and gripper down a bit early for efficiency
                            robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_READY);
                            robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_OPEN);

                            return Unit.INSTANCE;
                        })
                        .strafeTo(new Vector2d(stoneX, stoneY)) // strafe to nearest regular stone
                .build()
        );

        // pick up third stone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(900);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(stoneX, stoneY + 3)) // strafe left a bit to avoid knocking into other stones
                        .strafeTo(new Vector2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y)) // strafe to under blue bridge
                        .splineTo(new Pose2d(DUMP_CLOSE_X, DUMP_CLOSE_Y, 0)) // spline to closest dumping position
                        .strafeTo(new Vector2d(DUMP_CLOSE_X, DUMP_CLOSE_Y - 6)) // strafe closer to foundation (to right)
                .build()
        );

        // dump third stone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_STOWED);
        sleep(500);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

         */


        // get foundation gripper ready for pull
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_READY);

        // turn 90 deg right to grip foundation
        drive.turnSync(Math.toRadians(-90));

        // back up to grip foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse to back up
                        .lineTo(new Vector2d(DUMP_MID_X, DUMP_MID_Y - 12))
                        .build()
        );

        // grip foundation
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_DOWN);

        // pull foundation into building site
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FOUNDATION_END_POS) // spline to ending position after pulling foundation
                        .build()
        );

        // ungrip foundation
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_UP);

        // drive backwards into foundation for insurance and strafe closer to bridge before parking
        // to avoid hitting alliance partner
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse direction to move backwards
                        .lineTo(new Vector2d(FOUNDATION_END_X + 5, FOUNDATION_END_Y)) // drive backwards
                        .strafeTo(new Vector2d(FOUNDATION_END_X + 5, PARKED_Y)) // strafe closer to bridge
                        .build()
        );

        // park under blue bridge
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(PARKED_X, PARKED_Y)) // strafe to parking position
                        .build()
        );
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}