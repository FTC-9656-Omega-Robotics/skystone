package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.OmegaBotRR;

@Autonomous(group = "drive")

public class JustPark extends LinearOpMode {
    OmegaBotRR robot;
    SampleMecanumDriveREV drive;

    // pre-condition: robot is as close to bridge as possible AND facing the bridge
    final double DISTANCE = 10; // inches
    final long SLEEP_TIME = 28000; // 28 sec so that we park at end of auto

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        sleep(SLEEP_TIME);

        // move forward a certain distance to park under bridge
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(DISTANCE)
                        .build()
        );
    }
}
