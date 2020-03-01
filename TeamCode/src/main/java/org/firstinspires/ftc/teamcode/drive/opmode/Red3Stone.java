package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoBackend.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.OmegaBotRR;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


import java.util.Vector;

import kotlin.Unit;

@Autonomous(group = "drive")
public class Red3Stone extends LinearOpMode {
    OmegaBotRR robot;
    private OpenCvCamera phoneCam;
    private CustomSkystoneDetector skyStoneDetector;

    String skystonePosition = "none";
    double xPosition;
    double yPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        //initializes camera detection stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //gets the camera ready and views the skystones
        phoneCam.openCameraDevice();
        skyStoneDetector = new CustomSkystoneDetector();
        skyStoneDetector.useDefaults();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        // TODO: generally, just check if the position constants (stuff with final in front) are accurate

        final Pose2d ROBOT_INIT_POSITION = new Pose2d(-39,-63,0);

        // 1 is closest to bridge, 6 is closest to wall
        final int SKYSTONE_Y = -35;

        final Pose2d SKYSTONE_POS_1 = new Pose2d(-21, SKYSTONE_Y, 0); // testing
        final Pose2d SKYSTONE_POS_2 = new Pose2d(-29, SKYSTONE_Y, 0); // unverified
        final Pose2d SKYSTONE_POS_3 = new Pose2d(-37, SKYSTONE_Y, 0); // unverified
        final Pose2d SKYSTONE_POS_4 = new Pose2d(-45, SKYSTONE_Y, 0); // testing
        final Pose2d SKYSTONE_POS_5 = new Pose2d(-53, SKYSTONE_Y, 0); // unverified
        final Pose2d SKYSTONE_POS_6 = new Pose2d(-61, SKYSTONE_Y, 0); // unverified

        /* generalized, but may not work in practice
        final int SKYSTONE_1_X = -21;
        final int LENGTH_OF_STONE = 8;

        final Pose2d SKYSTONE_POS_1 = new Pose2d(SKYSTONE_1_X, SKYSTONE_Y, 0);
        final Pose2d SKYSTONE_POS_2 = new Pose2d(SKYSTONE_1_X  - (1 * LENGTH_OF_STONE), SKYSTONE_Y, 0);
        final Pose2d SKYSTONE_POS_3 = new Pose2d(SKYSTONE_1_X - (2 * LENGTH_OF_STONE), SKYSTONE_Y, 0);
        final Pose2d SKYSTONE_POS_4 = new Pose2d(SKYSTONE_1_X - (3 * LENGTH_OF_STONE), SKYSTONE_Y, 0);
        final Pose2d SKYSTONE_POS_5 = new Pose2d(SKYSTONE_1_X - (4 * LENGTH_OF_STONE), SKYSTONE_Y, 0);
        final Pose2d SKYSTONE_POS_6 = new Pose2d(SKYSTONE_1_X - (5 * LENGTH_OF_STONE), SKYSTONE_Y, 0);

        */

        // a bit of space between robot and neutral bridge
        final Pose2d UNDER_RED_BRIDGE_POS = new Pose2d(0, -40, 0);

        // parked position
        // TODO: check if 180 deg heading makes robot face wall that touches stones
        final Pose2d PARKED = new Pose2d(0, -36, Math.toRadians(180));

        // after gripping foundation, splines to this position to move foundation into building site
        // TODO: check if 180 deg heading makes robot face wall that touches stones
        final Pose2d FOUNDATION_POS = new Pose2d(35, -55, Math.toRadians(180));

        // far = close to wall, close = close to bridge
        final int DUMP_Y = -30;

        final Pose2d DUMP_POS_FAR = new Pose2d(60, DUMP_Y, 0);
        final Pose2d DUMP_POS_MID = new Pose2d(55, DUMP_Y, 0);
        final Pose2d DUMP_POS_CLOSE = new Pose2d(45, DUMP_Y, 0);

        // for testing: skystonePosWall is SKYSTONE_POS_4
        int skystoneWallX = -45;
        int skystoneWallY = SKYSTONE_Y;

        // for testing: skystonePosBridge is SKYSTONE_POS_1
        int skystoneBridgeX = -21;
        int skystoneBridgeY = SKYSTONE_Y;

        // actual code: initialize these variables when the camera figures out
        // what the skystonePosition is (see while loop below)
        // initialized for testing
        Pose2d skystonePosWall = SKYSTONE_POS_4; // position of skystone closest to the wall
        Pose2d skystonePosBridge = SKYSTONE_POS_1; // position of skystone closest to the bridge

        // initialized for testing
        Pose2d nearestStone = SKYSTONE_POS_2; // position of nearest regular stone to pick up

        double positionCorrector = 0;
        //All comments comment above what is being commented

        while (!isStopRequested() && !opModeIsActive()) {
            xPosition = skyStoneDetector.foundRectangle().x;
            yPosition = skyStoneDetector.foundRectangle().y;

            if (xPosition >= 180 || xPosition < 40) {
                skystonePosition = "right";
                //positionCorrector = 14;
            } else if (xPosition > 130) {//x = 12
                skystonePosition = "center";
                //positionCorrector = 8;
            } else {
                skystonePosition = "left";
                //positionCorrector = 0;
            }

            telemetry.addData("xPos", xPosition);
            telemetry.addData("yPos", yPosition);
            telemetry.addData("SkyStone Pos", skystonePosition);
            telemetry.update();
        }
        //TODO Find what numbers we need to set the positionCorrector to for each skystone position, then incorporate that into our spline code

        waitForStart();

        if (isStopRequested()) return;

        // before moving, get side back elbow and gripper ready
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED);

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // move to first skystone (closest to wall)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(skystoneWallX, skystoneWallY)) // strafe to first skystone
                .build()
        );

        // pick up first skystone
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(900);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // move away from stones a bit so that the gripped stone doesn't hit the other ones when robot moves
                    .strafeTo(new Vector2d(skystoneWallX, skystoneWallY - 3))
                    .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                    .splineTo(DUMP_POS_FAR) // spline to farthest dumping position
                .build()
        );

        // dump first skystone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // move to second skystone (closest to bridge)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .reverse() // reverse direction to go back to quarry
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                    .addMarker( () -> { // move side back elbow down a bit early for efficiency
                        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_READY);

                        return Unit.INSTANCE;
                    })
                    .strafeTo(new Vector2d(skystoneBridgeX, skystoneBridgeY)) // strafe to second skystone
                .build()
        );

        // pick up second skystone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(900);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                        .splineTo(DUMP_POS_MID) // spline to middle dumping position
                .build()
        );

    /*
        // dump second skystone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // move to third stone (a regular one)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse() // reverse direction to go back to quarry
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                        .addMarker( () -> { // move side back elbow down a bit early for efficiency
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_READY);

                            return Unit.INSTANCE;
                        })
                        .splineTo(nearestStone) // spline to nearest regular stone
                .build()
        );

        // pick up third stone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(900);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                        .splineTo(DUMP_POS_CLOSE) // spline to closest dumping position
                .build()
        );

        // dump third stone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // get foundation gripper ready for pull
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_READY);

        // move to grip foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // TODO: check if 90 deg is turning right or left
                        .splineTo(new Pose2d(45, DUMP_Y, Math.toRadians(90))) // turn 90 deg right
                        .splineTo(new Pose2d(45, DUMP_Y - 1, Math.toRadians(90))) // back up to be close enough to grip foundation
                .build()
        );

        // grip foundation
        robot.foundationGripper.setPosition(OmegaBotRR.FOUNDATION_GRIPPER_DOWN);

        // drive a bit closer to building site and turn another 90 deg right
        // to move foundation into building site
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FOUNDATION_POS) // spline to ending position after pulling foundation
                .build()
        );

        // park under red bridge
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(PARKED) // spline to parking position
                .build()
        );

         */
    }
}