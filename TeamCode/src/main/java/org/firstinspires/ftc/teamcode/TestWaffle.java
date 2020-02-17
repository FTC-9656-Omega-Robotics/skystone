package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestWaffle")
public class TestWaffle extends LinearOpMode {
    OmegaBot robot;
    OmegaPID drivePID;
    MotionMethods motionMethods;

    public void runOpMode(){
        robot = new OmegaBot(telemetry, hardwareMap);
        drivePID = robot.drivePID;
        motionMethods = new MotionMethods(robot, telemetry, this);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        motionMethods.strafe(0,80, 1);

        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(5,1);
        robot.drivetrain.reverseDirection();

        motionMethods.strafe(180,80,1);

        motionMethods.moveMotionProfile(10,1);
    }
}
