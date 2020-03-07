package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous (name = "Blue Stone Position Tester")
@Disabled
public class BlueStonePositionTester extends LinearOpMode {
    OmegaBotRR robot;
    SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        drive = new SampleMecanumDriveREV(hardwareMap);

        // robot's initial position
        final int INIT_X = -35;
        final int INIT_Y = 67;
        final Pose2d ROBOT_INIT_POSITION = new Pose2d(INIT_X, INIT_Y, Math.toRadians(180)); // 180 deg because robot faces wall that touches stones

        // TODO: tune the following coordinates so that when strafing to that position, the gripper is in the center of the block
        // Coordinates are given to pick up with front gripper
        // skystone 1 is closest to bridge, skystone 6 is closest to wall
        int[] SKYSTONE_X = {-18, -26, -34, -44, -52, -60};
        // index:             0   1    2    3    4    5
        // skystone #:        1   2    3    4    5    6

        final int y = 38;
        int[] SKYSTONE_Y = {y, y, y, y, y, y}; // should theoretically have same y coordinate
        // index:           0  1  2  3  4  5
        // skystone #:      1  2  3  4  5  6

        // stone # that we are currently testing (NOT the index)
        int stone = 6;


        waitForStart();

        if (isStopRequested()) return;

        // set initial position
        drive.setPoseEstimate(ROBOT_INIT_POSITION);

        // before going to skystone, get side front elbow and gripper ready
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_READY);
        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_READY);

        // strafe from init position to skystone
        // pickup block at that skystone and then put it back down
        testPosition(stone, SKYSTONE_X, SKYSTONE_Y, INIT_X, INIT_Y);
    }

    /**
     * Strafe from init position to skystone at a given position
     * At that skystone, pickup the block and then put it back down
     *
     * @param stone       stone number that is currently being tested (from 1-6)
     * @param SKYSTONE_X  array of x coordinates of skystones
     * @param SKYSTONE_Y  array of y coordinates of skystones
     * @param INIT_X      x coordinate of init position
     * @param INIT_Y      y coordinate of init position
     */
    public void testPosition(int stone, int[] SKYSTONE_X, int[] SKYSTONE_Y, int INIT_X, int INIT_Y) {
        // strafe to skystone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(SKYSTONE_X[stone - 1], SKYSTONE_Y[stone - 1]))
                        .build()
        );

        // pick up skystone and put it back down
        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);

        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(1100);

        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);

        robot.sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_OPEN);
        sleep(500);

        robot.sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_READY);
        sleep(500);

        // strafe back to init position
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        // strafe left a bit to avoid knocking into other stones
                        .strafeTo(new Vector2d(SKYSTONE_X[stone - 1], SKYSTONE_Y[stone - 1] + 2))

                        // strafe back to init position
                        .strafeTo(new Vector2d(INIT_X, INIT_Y))
                        .build()
        );
    }
}
