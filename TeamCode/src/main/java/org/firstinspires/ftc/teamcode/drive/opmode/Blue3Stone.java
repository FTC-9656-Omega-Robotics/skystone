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


import kotlin.Unit;

@Autonomous(group = "drive")
public class Blue3Stone extends LinearOpMode {
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
                positionCorrector = 0;
            }

            telemetry.addData("xPos", xPosition);
            telemetry.addData("yPos", yPosition);
            telemetry.addData("SkyStone Pos", skystonePosition);
            telemetry.update();
        }
        //TODO Find what numbers we need to set the positionCorrector to for each skystone position, then incorporate that into our spline code


        waitForStart();

        if (isStopRequested()) return;

        // set initial position of the robot (x- and y-coordinates and heading)
        drive.setPoseEstimate(new Pose2d(-39,-63,0));

        // commands inside the trajectory run one after another
        drive.followTrajectorySync(
                drive.trajectoryBuilder()//new TrajectoryBuilder(new Pose2d(0,0,0), drive.getConstraints())
                        .strafeTo(new Vector2d(-39,-39)) // strafe to coordinates (without turning)
                        .build()
        );
        drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                        .splineTo(new Pose2d(55,-35, 0)) // spline to coordinates (turns)
<<<<<<< HEAD
                 .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()

=======
                        .reverse() // makes robot go in reverse direction (rather than turning around to go to new Pose2d)
                        .splineTo(new Pose2d(-52,-39,0))//goes to pick up second block
                        .reverse()//reverses the reverse so it's normal again
                         .splineTo(new Pose2d(49,-35,0))//goes to drop of second block
>>>>>>> 2c09ac9bd97c4b172278917d0defe16760ba8545
                        .build()

        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()//reverses the robot to go pick up the third block
                        .splineTo(new Pose2d(-20,-35,0))//goes to pick up third block
                        .reverse()//reverses the previous reverse to get it back to normal
                        .splineTo(new Pose2d(40,-35,0))//goes to drop off 3d block
                        .build()
        );



    }
}
