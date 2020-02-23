package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OmegaBotRR;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

@Autonomous(group = "drive")
public class Blue3Stone extends LinearOpMode {
    OmegaBotRR robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OmegaBotRR(telemetry, hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-39,-63,0));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()//new TrajectoryBuilder(new Pose2d(0,0,0), drive.getConstraints())
                        .strafeTo(new Vector2d(-39,-39))
                        .addMarker(() -> {
                            drive.setPoseEstimate(new Pose2d(-39,-27,0));
                            //add servos
                            return Unit.INSTANCE;
                        })
                        .splineTo(new Pose2d(55,-35, 0))
                        .reverse()
                        .splineTo(new Pose2d(-52,-39,0))
                        .build()
        );


    }
}
