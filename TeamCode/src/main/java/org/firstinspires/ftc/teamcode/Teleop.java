package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop", group = "prototype")
@Disabled

public class Teleop extends OpMode {
    OmegaBot robot;
    double maxSpeed = .25;

    public void init() {
        robot = new OmegaBot(telemetry, hardwareMap);
    }

    public void loop() {
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;
        double temp = forward * Math.cos(Math.toRadians(robot.getAngle())) - right * Math.sin(Math.toRadians(robot.getAngle()));
        right = forward * Math.sin(Math.toRadians(robot.getAngle())) + right * Math.cos(Math.toRadians(robot.getAngle()));
        forward = temp;

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
            front_left /= (max/maxSpeed);
            front_right /= (max/maxSpeed);
            rear_left /= (max/maxSpeed);
            rear_right /= (max/maxSpeed);
        }

        robot.frontLeft.setPower(front_left);
        robot.frontRight.setPower(front_right);
        robot.backLeft.setPower(rear_left);
        robot.backRight.setPower(rear_right);

        telemetry.addData("angle: ", robot.getAngle());

        if (gamepad2.a) {
            robot.arm.setPower(0.5);
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(1);
        } else if (gamepad2.y) {
            robot.arm.setPower(-.5);

        } else {
            robot.arm.setPower(0);
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

         if (gamepad2.right_bumper) {
             robot.leftIntake.setPower(-1);
             robot.rightIntake.setPower(1);
         } else if (gamepad2.right_trigger > 0.5) {
             robot.leftIntake.setPower(1);
             robot.rightIntake.setPower(-1);
         } else {
             robot.leftIntake.setPower(0);
             robot.rightIntake.setPower(0);
         }

        if(gamepad2.b) {
            //robot.extension.setPower(0.5);
        } else if (gamepad2.x) {
            //robot.extension.setPower(-.5);
        } else {
            //robot.extension.setPower(0);
        }
    }
}


