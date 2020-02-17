package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "StraightTest")
//@Disabled
public class StraightTest extends LinearOpMode {
    OmegaBot robot;
    MotionMethods motionMethods;

    public void runOpMode(){
        robot = new OmegaBot(telemetry, hardwareMap);
        motionMethods = new MotionMethods(robot, telemetry, this);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        robot.arm.setTargetPosition(-400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        //teleop arm up

        motionMethods.moveMotionProfile(50,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfileReverse(50,1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        motionMethods.moveMotionProfile(50,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfileReverse(50,1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        motionMethods.moveMotionProfile(50,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfileReverse(50,1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        motionMethods.moveMotionProfile(50,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfileReverse(50,1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(0,1);
    }
}
