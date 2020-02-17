package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="redFoundation")
public class redFoundation extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double robotSpeed = 0.45;
    OmegaBot robot;
    OmegaPID drivePID;
    MotionMethods motionMethods;

    public void runOpMode() {
        robot = new OmegaBot(telemetry, hardwareMap);
        drivePID = robot.drivePID;
        motionMethods = new MotionMethods(robot, telemetry, this);
        /*
         */
        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        motionMethods.turnUsingPIDVoltage(90, .5);
        robot.drivetrain.reverseDirection();
        motionMethods.movePID(32, .5);

        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltage(-90, .5);
        motionMethods.movePID(75, .5);
        motionMethods.turnUsingPIDVoltage(90, .5);
        robot.drivetrain.reverseDirection();
        motionMethods.movePID(23, .5);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltage(-90, .5);
        robot.drivetrain.reverseDirection();
        motionMethods.movePID(50, .5);








        //GYRO SETUP
        runtime.reset();
    }
}
