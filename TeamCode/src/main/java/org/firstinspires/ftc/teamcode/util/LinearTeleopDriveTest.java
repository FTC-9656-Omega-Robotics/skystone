package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OmegaBot;

@TeleOp (name = "Linear Teleop Drive Test")
public class LinearTeleopDriveTest extends LinearOpMode {
    private OmegaBot robot;
    private ElapsedTime runtime;
    private double maxSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OmegaBot(telemetry, hardwareMap); // initialize hardware
        waitForStart();

        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // while running this opmode, continuously check gamepads for input
        while (opModeIsActive()) {
            drivetrainProcess();
        }
    }

    public void drivetrainProcess() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x * .75;

        // cubic drive - instead of proportional power to motors, cube input from joysticks
        forward = Math.pow(forward, 3);
        right = Math.pow(right, 3);
        clockwise = Math.pow(clockwise, 3);

        //double temp = forward * Math.cos(Math.toRadians(robot.getAngle())) - right * Math.sin(Math.toRadians(robot.getAngle()));
        //right = forward * Math.sin(Math.toRadians(robot.getAngle())) + right * Math.cos(Math.toRadians(robot.getAngle()));
        //forward = temp;

        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        //double FrontLeftVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        //double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
        //double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        //double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

        double max = Math.abs(front_left);
        if (Math.abs(front_right) > max) max = Math.abs(front_right);
        if (Math.abs(rear_left) > max) max = Math.abs(rear_left);
        if (Math.abs(rear_right) > max) max = Math.abs(rear_right);

        if (max > maxSpeed) {
            front_left /= (max / maxSpeed);
            front_right /= (max / maxSpeed);
            rear_left /= (max / maxSpeed);
            rear_right /= (max / maxSpeed);
        }

        if (gamepad1.a) { //if a strafe left
            robot.frontLeft.setPower(-1);
            robot.frontRight.setPower(1);
            robot.backLeft.setPower(1);
            robot.backRight.setPower(-1);
        } else if (gamepad1.b) { //if b strafe right
            robot.frontLeft.setPower(1);
            robot.frontRight.setPower(-1);
            robot.backLeft.setPower(-1);
            robot.backRight.setPower(1);
        } else { //otherwise joysticks
            robot.frontLeft.setPower(front_left);
            robot.frontRight.setPower(front_right);
            robot.backLeft.setPower(rear_left);
            robot.backRight.setPower(rear_right);
        }
    }
}
