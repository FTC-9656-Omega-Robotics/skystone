package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous (name = "Stone Position Tester")
public class StonePositionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot and drivetrain
        OmegaBotRR robot = new OmegaBotRR(telemetry, hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        // robot's initial position
        final int INIT_X = -39;
        final int INIT_Y = -63;
        final Pose2d ROBOT_INIT_POSITION = new Pose2d(INIT_X, INIT_Y, 0);

        // TODO: tune the following coordinates so that when strafing to that position, the gripper is in the center of the block
        // skystone 1 is closest to bridge, skystone 6 is closest to wall
        int[] SKYSTONE_X = {-21, -29, -37, -45, -53, -61};
               // index:      0   1    2    3    4    5
               // skystone #: 1   2    3    4    5    6

        final int y = -35;
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

        // strafe from init position to each skystone
        // at each skystone, pickup block and then put it back down
        for (int i = 0; i < SKYSTONE_X.length; i++) {
            // strafe to skystone
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(SKYSTONE_X[i], SKYSTONE_Y[i]))
                            .build()
            );

            // pick up skystone and put it back down
            robot.sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
            sleep(750);

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
                            .strafeTo(new Vector2d(SKYSTONE_X[i], SKYSTONE_Y[i] - 2))

                            // strafe back to init position
                            .strafeTo(new Vector2d(INIT_X, INIT_Y))
                            .build()
            );

        }
    }
}
