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
        // initialize robot and drivetrain
        robot = new OmegaBotRR(telemetry, hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        // set initial position of the robot (x- and y-coordinates and heading)
        drive.setPoseEstimate(new Pose2d(-39,-63,0));

        // commands inside the trajectory run one after another
        drive.followTrajectorySync(
                drive.trajectoryBuilder()//new TrajectoryBuilder(new Pose2d(0,0,0), drive.getConstraints())
                        .strafeTo(new Vector2d(-39,-39)) // strafe to coordinates (without turning)
                        .addMarker(() -> { // addMarker to do things while following the trajectory
                            drive.setPoseEstimate(new Pose2d(-39,-27,0));
                            //add servos
                            return Unit.INSTANCE;
                        })
                        .splineTo(new Pose2d(55,-35, 0)) // spline to coordinates (turns)
                        .reverse() // makes robot go in reverse direction (rather than turning around to go to new Pose2d)
                        .splineTo(new Pose2d(-52,-39,0))
                        .build() // builds the path that I coded above
        );


    }
}
