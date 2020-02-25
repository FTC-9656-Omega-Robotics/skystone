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

    Pose2d ROBOT_INIT_POSITION = new Pose2d(-39,-63,0);

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

        // 1 is closest to bridge, 6 is closest to wall
        // each stone is 8 in long
        final Pose2d SKYSTONE_POS_1 = new Pose2d(-29, -39, 0);
        final Pose2d SKYSTONE_POS_2 = new Pose2d(-37, -39, 0);
        final Pose2d SKYSTONE_POS_3 = new Pose2d(-45, -39, 0);
        final Pose2d SKYSTONE_POS_4 = new Pose2d(-53, -39, 0);
        final Pose2d SKYSTONE_POS_5 = new Pose2d(-61, -39, 0);
        final Pose2d SKYSTONE_POS_6 = new Pose2d(-69, -39, 0);

        // a bit of space between robot and neutral bridge
        final Pose2d UNDER_RED_BRIDGE_POS = new Pose2d(0, -40, 0);

        // parked position
        // TODO: check if 180 deg heading makes robot face wall that touches stones
        final Pose2d PARKED = new Pose2d(0, -36, Math.toRadians(180));

        // after gripping foundation, splines to this position to move foundation into building site
        // TODO: check if 180 deg heading makes robot face wall that touches stones
        final Pose2d FOUNDATION_POS = new Pose2d(35, -55, Math.toRadians(180));

        // far = close to wall, close = close to bridge
        final Pose2d DUMP_POS_FAR = new Pose2d(60, -35, 0);
        final Pose2d DUMP_POS_MID = new Pose2d(55, -35, 0);
        final Pose2d DUMP_POS_CLOSE = new Pose2d(45, -35, 0);

        // for testing: skystonePosWall is SKYSTONE_POS_4
        int skystoneWallX = -53;
        int skystoneWallY = -39;

        // for testing: skystonePosBridge is SKYSTONE_POS_1
        int skystoneBridgeX = -29;
        int skystoneBridgeY = -39;

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

        // move side back elbow down before moving
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // move to first skystone (closest to wall)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(skystonePosWall) // spline to first skystone
                .build()
        );

        // pick up first skystone
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                    .splineTo(DUMP_POS_FAR) // spline to farthest dumping position
                .build()
        );

        // dump first skystone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // reverse direction to go back to quarry
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                .build()
        );

        // move to second skystone (closest to bridge)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                    .addMarker( () -> { // move side back elbow down a bit early for efficiency
                        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_READY);

                        return Unit.INSTANCE;
                    })
                    .splineTo(skystonePosBridge) // spline to second skystone
                .build()
        );

        // pick up second skystone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // reverse direction to go back to building zone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                .build()
        );

        // move to foundation to dump
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                        .splineTo(DUMP_POS_MID) // spline to middle dumping position
                .build()
        );

        // dump second skystone
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // reverse direction to go back to quarry
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                .build()
        );

        // move to third stone (a regular one)
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
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
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // reverse direction to go back to building zone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                .build()
        );

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
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
        sleep(500);
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        // turn to grip foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // turn 90 deg right to grip foundation
                        // TODO: check if 90 deg is turning right or left
                        // TODO: If any changes made to x and y of dumping pos constants, change x and y here accordingly since they're based on DUMP_POS_CLOSE
                        .splineTo(new Pose2d(45, -35, Math.toRadians(90)))
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


        /*
            old code below, functional as of Tue 2/25/2020
            TODO: remove this old code if above code works relatively well/better

        // commands inside the trajectory run one after another
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(skystoneWallX, skystoneWallY)) // strafe to first skystone (closest to wall)
//                        .addMarker( () -> {
//                            // pick up first skystone
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
//                            sleep(500);
//                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
//                            sleep(500);
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
//                            sleep(500);
//
//                            return Unit.INSTANCE;
//                        })
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline to under red bridge
                        .splineTo(DUMP_POS_FAR)// spline to foundation far dumping position
//                        .addMarker( () -> {
//                            // dump skystone
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
//                            sleep(500);
//                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
//                            sleep(500);
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
//                            sleep(500);
//
//                            return Unit.INSTANCE;
//                        })
                        .reverse()// reverse direction to go back for stone
                        .splineTo(UNDER_RED_BRIDGE_POS) // spline back under bridge
//                        .addMarker( () -> {
//                            // put side front elbow in ready position
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_READY);
//                            sleep(500);
//
//                            return Unit.INSTANCE;
//                        })
                        .splineTo(skystonePosBridge) // spline to second skystone (closest to bridge)
//                        .addMarker( () -> {
////                            // pick up second skystone
////                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
////                            sleep(500);
////                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
////                            sleep(500);
////                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
////                            sleep(500);
////
////                            return Unit.INSTANCE;
////                        })
                        .reverse() //reverses the reverse so it goes forward again
                        .splineTo(UNDER_RED_BRIDGE_POS)// spline to under bridge
                        .splineTo(DUMP_POS_MID) // splines to foundation to dump skystone
//                        .addMarker( () -> {
//                            // dump second skystone
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
//                            sleep(500);
//                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
//                            sleep(500);
//                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
//                            sleep(500);
//
//                            return Unit.INSTANCE;
//                        })
                        .reverse() // reverse direction to go back for stone
                        .splineTo(SKYSTONE_POS_1)
                        .reverse()
                        .splineTo(UNDER_RED_BRIDGE_POS)
                        .splineTo(DUMP_POS_CLOSE)
                        // do third (regular) stone
                        // move foundation
                        // park under skybridge
                        .build()


        );

        drive.turnSync(Math.toRadians(90));

        */
    }
}