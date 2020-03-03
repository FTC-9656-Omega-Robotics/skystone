package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous (name = "Stone Position Tester")
public class StonePositionTester extends LinearOpMode {
    OmegaBotRR robot;
    SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        drive = new SampleMecanumDriveREV(hardwareMap);

        // robot's initial position
        final int INIT_X = -35;
        final int INIT_Y = -59;
        final Pose2d ROBOT_INIT_POSITION = new Pose2d(INIT_X, INIT_Y, 0);

        // TODO: tune the following coordinates so that when strafing to that position, the gripper is in the center of the block
        // Coordinates are given to pick up with back gripper
        // skystone 1 is closest to bridge, skystone 6 is closest to wall
        int[] SKYSTONE_X = {-14, -22, -30, -38, -46, -54};
               // index:      0   1    2    3    4    5
               // skystone #: 1   2    3    4    5    6

        final int y = -28;
        int[] SKYSTONE_Y = {y, y, y, y, y, y}; // should theoretically have same y coordinate
        // index:           0  1  2  3  4  5
        // skystone #:      1  2  3  4  5  6


        waitForStart();

        if (isStopRequested()) return;

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // before going to skystone, get side back elbow and gripper ready
        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED);

        // strafe from init position to skystone
        // pickup block at that skystone and then put it back down
        testPosition(5, SKYSTONE_X, SKYSTONE_Y, INIT_X, INIT_Y);
    }

    // strafe from init position to each skystone
    // at each skystone, pickup block and then put it back down
    public void testPosition(int stone, int[] SKYSTONE_X, int[] SKYSTONE_Y, int INIT_X, int INIT_Y) {
        // strafe to skystone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(SKYSTONE_X[stone], SKYSTONE_Y[stone]))
                        .build()
        );

        // pick up skystone and put it back down
        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(1100);

        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);

        robot.sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);

        robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED);
        sleep(500);

        // strafe back to init position
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // strafe right a bit to avoid knocking into other stones
                        .strafeTo(new Vector2d(SKYSTONE_X[stone], SKYSTONE_Y[stone] - 2))

                        // strafe back to init position
                        .strafeTo(new Vector2d(INIT_X, INIT_Y))
                        .build()
        );
    }
}
