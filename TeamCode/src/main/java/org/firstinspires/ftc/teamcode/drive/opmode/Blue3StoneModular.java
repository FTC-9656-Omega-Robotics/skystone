package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoBackend.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.OmegaBotRR;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import kotlin.Unit;

@Autonomous (name = "Blue 3 Stone Modular")
public class Blue3StoneModular extends LinearOpMode {
    // ----------------- HARDWARE --------------------------------
    OmegaBotRR robot;
    SampleMecanumDriveBase drive;


    // ----------------- COMPUTER VISION -------------------------
    private OpenCvCamera phoneCam;
    private CustomSkystoneDetector skyStoneDetector;

    // position of skystone closest to the bridge (none, 1, 2, or 3)
    String skystonePosition = "none";
    double xPosition;
    double yPosition;


    // ----------------- CONSTANT POSITIONS ----------------------

    // initial position of the robot
    final int INIT_X = -35;
    final int INIT_Y = 59;
    final Pose2d ROBOT_INIT_POSITION = new Pose2d(INIT_X, INIT_Y, 0);

    // skystone coordinates
    // 1 is closest to bridge, 6 is closest to wall
    // NOTE: these coordinates should only be used in pairs (1-4, 2-5, 3-6) when running full auto paths
    // they may not actually represent where the skystones are; they're just coordinates that work
    // when tuning each of the 3 blue autonomous paths
    //  TODO: tune stone positions since side front gripper is grabbing blocks
    final int SKYSTONE_1_X = -8;
    final int SKYSTONE_2_X = -16;
    final int SKYSTONE_3_X = -26;
    final int SKYSTONE_4_X = -38;
    final int SKYSTONE_5_X = -46;
    final int SKYSTONE_6_X = -54;

    final int SKYSTONE_1_Y = 26;
    final int SKYSTONE_2_Y = 25;
    final int SKYSTONE_3_Y = 25;
    final int SKYSTONE_4_Y = 28;
    final int SKYSTONE_5_Y = 28;
    final int SKYSTONE_6_Y = 28;

    // a bit of space between robot and neutral bridge
    final int UNDER_BLUE_BRIDGE_X = 0;
    final int UNDER_BLUE_BRIDGE_Y = 32;

    // parked position coordinates
    final int PARKED_X = 4;
    final int PARKED_Y = 22;

    // after gripping foundation, splines to this position to move foundation into building site
    final Pose2d FOUNDATION_END_POS = new Pose2d(40, -50, Math.toRadians(180));

    // dump position coordinates
    // far = close to wall, close = close to bridge
    final int DUMP_FAR_X = 68;
    final int DUMP_MID_X = 65;
    final int DUMP_CLOSE_X = 60;

    final int DUMP_FAR_Y = 25;
    final int DUMP_MID_Y = 25;
    final int DUMP_CLOSE_Y = 25;

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

    // heading in degrees of robot for the first time it splines
    // under the bridge (after picking up first skystone)
    int bridgeAngle;


    @Override
    public void runOpMode() throws InterruptedException {
        // ----------------- INITIALIZATION --------------------

        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        drive = new SampleMecanumDriveREV(hardwareMap);

        //initializes camera detection stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //gets the camera ready and views the skystones
        phoneCam.openCameraDevice();
        skyStoneDetector = new CustomSkystoneDetector();
        skyStoneDetector.useDefaults();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);


        // ------------ CHOOSING AN AUTO PATH --------------

        // use CV to detect location of the skystone
        while (!isStopRequested() && !opModeIsActive()) {
            xPosition = skyStoneDetector.foundRectangle().x;
            yPosition = skyStoneDetector.foundRectangle().y;

            if (xPosition >= 180 || xPosition < 40) {
                skystonePosition = "1";

                // skystoneWallX = [SKYSTONE_POS_X];
                // skystoneWallY = [SKYSTONE_POS_Y];

                // skystoneBridgeX = [SKYSTONE_POS + 3_X];
                // skystoneBridgeX = [SKYSTONE_POS + 3_X];

                // stoneX = [?]; // stone [#]
                // stoneY = [?]; // stone [#]
            } else if (xPosition > 130) {//x = 12
                skystonePosition = "2";

                // skystoneWallX = [SKYSTONE_POS_X];
                // skystoneWallY = [SKYSTONE_POS_Y];

                // skystoneBridgeX = [SKYSTONE_POS + 3_X];
                // skystoneBridgeX = [SKYSTONE_POS + 3_X];

                // stoneX = [?]; // stone [#]
                // stoneY = [?]; // stone [#]
            } else {
                skystonePosition = "3";

                // skystoneWallX = [SKYSTONE_POS_X];
                // skystoneWallY = [SKYSTONE_POS_Y];

                // skystoneBridgeX = [SKYSTONE_POS + 3_X];
                // skystoneBridgeX = [SKYSTONE_POS + 3_X];

                // stoneX = [?]; // stone [#]
                // stoneY = [?]; // stone [#]
            }

            telemetry.addData("xPos", xPosition);
            telemetry.addData("yPos", yPosition);
            telemetry.addData("SkyStone Pos", skystonePosition);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        // --------- EXECUTE CHOSEN AUTO PATH ----------

        // for testing only
        skystoneBridgeX = SKYSTONE_1_X;
        skystoneBridgeY = SKYSTONE_1_Y;

        skystoneWallX = SKYSTONE_4_X;
        skystoneWallY = SKYSTONE_4_Y;

        stoneX = 0; // null
        stoneY = 0; // null

        bridgeAngle = 10;

        executeAutoPath(skystoneBridgeX, skystoneBridgeY, skystoneWallX, skystoneWallY, stoneX, stoneY, bridgeAngle);
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
     * @param bridgeAngle      angle, in degrees, of the heading of the robot when it moves under the bridge the first time
     */
    public void executeAutoPath(int skystoneBridgeX, int skystoneBridgeY, int skystoneWallX, int skystoneWallY, int stoneX, int stoneY, int bridgeAngle) {
        // before moving, get side front elbow and gripper ready
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_OPEN);

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // move to first skystone (closest to wall)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(skystoneWallX, skystoneWallY)) // strafe to first skystone
                        .build()
        );

        // pick up first skystone
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(900);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // move away from stones a bit so that the gripped stone doesn't hit the other ones when robot moves
                        .strafeTo(new Vector2d(skystoneWallX, skystoneWallY + 3))
                        .splineTo(new Pose2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y, Math.toRadians(bridgeAngle))) // spline to under blue bridge
                        .splineTo(new Pose2d(DUMP_FAR_X, DUMP_FAR_Y, 0)) // spline to farthest dumping position
                        .build()
        );

        // dump first skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_STOWED);
        sleep(500);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

        // move to second skystone (closest to bridge)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse direction to go back to quarry
                        .splineTo(new Pose2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y, 0)) // spline to under blue bridge
                        .addMarker( () -> {
                            // move side front elbow gripper down a bit early for efficiency
                            robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_READY);
                            robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_OPEN);

                            return Unit.INSTANCE;
                        })
                        .strafeTo(new Vector2d(skystoneBridgeX, skystoneBridgeY)) // strafe to second skystone
                        .build()
        );

        // pick up second skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(900);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // move away from stones a bit so that the gripped stone doesn't hit the other ones when robot moves
                        .strafeTo(new Vector2d(skystoneBridgeX, skystoneBridgeY + 3))
                        .strafeTo(new Vector2d(UNDER_BLUE_BRIDGE_X, UNDER_BLUE_BRIDGE_Y)) // strafe to under blue bridge
                        .splineTo(new Pose2d(DUMP_MID_X, DUMP_MID_Y, 0)) // spline to middle dumping position
                        .strafeTo(new Vector2d(DUMP_MID_X, DUMP_MID_Y + 6)) // strafe closer to foundation (to left)
                        .build()
        );


        // dump second skystone
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_STOWED);
        sleep(500);
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

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
        drive.turnSync(Math.toRadians(90));

        // back up to grip foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse to back up
                        .lineTo(new Vector2d(DUMP_MID_X, DUMP_MID_Y - 12))
                        .build()
        );

        // grip foundation
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_DOWN);

        // drive a bit closer to building site and turn another 90 deg right
        // to move foundation into building site
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FOUNDATION_END_POS) // spline to ending position after pulling foundation
                        .build()
        );

        // ungrip foundation
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_UP);
        sleep(900);

        // park under blue bridge
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(PARKED_X, PARKED_Y)) // strafe to parking position
                        .build()
        );
    }
}