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
public class Blue3Stone extends LinearOpMode {
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




        //TODO: rename constants to all caps, and make skystone pos and dumping pos arrays
        //TODO change constants to x and y ints instead of poses (bc strafe requires vectors)

        // 1 is closest to bridge, 6 is closest to wall
        // each stone is 8 in long
        final Pose2d skystonePos1 = new Pose2d(-29, -39, 0);
        final Pose2d skystonePos2 = new Pose2d(-37, -39, 0);
        final Pose2d skystonePos3 = new Pose2d(-45, -39, 0);
        final Pose2d skystonePos4 = new Pose2d(-53, -39, 0);
        final Pose2d skystonePos5 = new Pose2d(-61, -39, 0);
        final Pose2d skystonePos6 = new Pose2d(-69, -39, 0);

        // a bit of space between robot and neutral bridge
        final Pose2d underRedBridgePos = new Pose2d(0, -35, 0);

        // far = close to wall, close = close to bridge
        final Pose2d dumpingPosFar = new Pose2d(60, -35, 0);
        final Pose2d dumpingPosMid = new Pose2d(55, -35, 0);
        final Pose2d dumpingPosClose = new Pose2d(45, -35, 0);

        // for testing: skystonePosWall is skystonePos4
        int skystoneWallX = -53;
        int skystoneWallY = -39;

        // for testing: skystonePosBridge is skystonePos1
        int skystoneBridgeX = -29;
        int skystoneBridgeY = -39;

        // initialize these variables when the camera figures out
        // what the skystonePosition is (see while loop below)
        Pose2d skystonePosWall = skystonePos4; // position of skystone closest to the wall
        Pose2d skystonePosBridge = skystonePos1; // position of skystone closest to the bridge

        Pose2d nearestStone = skystonePos2; // testing value only, same for 2 vars above

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

        /*
        3 stone auto code blue side:

        1. Move to skystonePositionWall
        2. Pick up skystone there
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        3. Move next to foundation (set a constant to that Pose2d position)
        4. Dump skystone on foundation
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        5. Move to skystonePositionBridge
            - put elbow in ready position
        6. Pick up skystone there
            robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
            robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        7. Move back to foundation (set a constant to that Pose2d position)
        8. Dump skystone on foundation
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        9. Move to closest regular stone (set a constant to that Pose2d position)
            - put elbow in ready position
        10. Pick up stone there
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        11. Dump skystone on foundation
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        12. Pull foundation into building zone
            // move closer to foundation?
            robot.foundationGrippers.setPosition(OmegaBotRR.FOUNDATION_GRIPPERS_DOWN);
            // pull foundation into building zone by turning smartly
            robot.foundationGrippers.setPosition(OmegaBotRR.FOUNDATION_GRIPPERS_UP);
        13. Park under skybridge (set a constant to that Pose2d position)
         */

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // commands inside the trajectory run one after another
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(skystoneWallX, skystoneWallY)) // strafe to first skystone (closest to wall)
                        .addMarker( () -> {
                            // pick up first skystone
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
                            sleep(500);
                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
                            sleep(500);
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
                            sleep(500);

                            return Unit.INSTANCE;
                        })
                        .splineTo(underRedBridgePos) // spline to under red bridge
                        .splineTo(dumpingPosFar)// spline to foundation far dumping position
                        .addMarker( () -> {
                            // dump skystone
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
                            sleep(500);
                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
                            sleep(500);
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
                            sleep(500);

                            return Unit.INSTANCE;
                        })
                        .reverse()// reverse direction to go back for stone
                        .splineTo(underRedBridgePos) // spline back under bridge
                        .addMarker( () -> {
                            // put side front elbow in ready position
                            robot.sideBackElbow.setPosition(0.15); // 0.15 is testing for ready pos, TODO make constant
                            sleep(500);

                            return Unit.INSTANCE;
                        })
                        .splineTo(skystonePosBridge) // spline to second skystone (closest to bridge)
                        .addMarker( () -> {
                            // pick up second skystone
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
                            sleep(500);
                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
                            sleep(500);
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
                            sleep(500);

                            return Unit.INSTANCE;
                        })
                        .reverse() //reverses the reverse so it goes forward again
                        .splineTo(underRedBridgePos)// spline to under bridge
                        .splineTo(dumpingPosMid) // splines to foundation to dump skystone
                        .addMarker( () -> {
                            // dump second skystone
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
                            sleep(500);
                            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_OPEN);
                            sleep(500);
                            robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
                            sleep(500);

                            return Unit.INSTANCE;
                        })
                        .reverse() // reverse direction to go back for stone
                        // do third (regular) stone
                        .build()

        );

        //TODO split up trajectory above into smaller ones to avoid emergency stop error
        /*

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()//reverses the robot to go pick up the third block
                        .splineTo(new Pose2d(-20,-35,0))//goes to pick up third block
                        .reverse()//reverses the previous reverse to get it back to normal
                        .splineTo(new Pose2d(40,-35,0))//goes to drop off 3d block
                        .build()
        );
        */
    }
}