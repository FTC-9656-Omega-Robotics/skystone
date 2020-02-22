package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OmegaBot;

@TeleOp(name = "LinearTeleop", group = "prototype")
public class LinearTeleop extends LinearOpMode {
    private OmegaBot robot;
    private ElapsedTime runtime;
    double maxSpeed = 1;
    boolean intakeOn = true;
    int armPos = 0;
    int count = 0;

    public void runOpMode() {
        robot = new OmegaBot(telemetry,hardwareMap);//initializing hardware
        waitForStart();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot.leftIntake.setPower(1);
        robot.rightIntake.setPower(-1);

        // while running this opmode, continuously check gamepads for input
        while(opModeIsActive()){
            drivetrainProcess();
            armProcess();
            servoProcess();
            intakeProcessIn();
            intakeProcessOut();
            foundationGrippers();
            //flagWave();

            telemetry.addData("armPos",robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }

    public void moveArm(double degrees) {
        double maxVel = 15 * 360 / 60000; // 15 is the rotations per minute, 360 is the degrees per rotation, 60000 is the number of milliseconds in a minute
        double macAcc = maxVel / 1300; //1300 is the number of milliseconds it takes to accelerate to full speed
        MotionProfileGenerator generator = new MotionProfileGenerator(maxVel, macAcc);
        double[] motionProfile = generator.generateProfile(degrees);
        double[] distanceProfile = generator.generateDistanceProfile(motionProfile);
        runtime.reset();
        while(runtime.milliseconds() < motionProfile.length && opModeIsActive()){
            robot.arm.setPower(motionProfile[(int)runtime.milliseconds()]/maxVel);//TODO: use the distance profile + encoders to pid up in dis bicth
        }
    }

    public void flagWave() {
        if (count < 50) {
            robot.sideBackElbow.setPosition(0.3);
            count++;
        } else if (count < 100) {
            robot.sideBackElbow.setPosition(0.35);
            count++;
        } else {
            count = 0;
        }
    }

    public void servoProcess() {
        if (gamepad2.x) {
            robot.blockRotator.setPosition(OmegaBot.BLOCK_ROTATOR_STRAIGHT);
        } else if (gamepad2.y) {
            robot.blockRotator.setPosition(OmegaBot.BLOCK_ROTATOR_ROTATED);
        }

        if (gamepad2.right_bumper) {
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_OPEN);
        } else if (gamepad2.left_bumper) {
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_CLOSED);
        }

        if (gamepad1.x) {
            robot.sideBackGripper.setPosition(OmegaBot.SIDE_BACK_GRIPPER_CLOSED);
        } else if (gamepad1.y) {
            robot.sideBackGripper.setPosition(OmegaBot.SIDE_BACK_GRIPPER_OPEN);
        }
    }

    public void foundationGrippers() {
        if (gamepad1.left_trigger > 0.5) {
            robot.foundationGripper.setPosition(OmegaBot.FOUNDATION_GRIPPER_DOWN);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.foundationGripper.setPosition(OmegaBot.FOUNDATION_GRIPPER_UP);
        }
    }

    public void armProcess() {
        double power = 0.5;
        boolean grabbingBlock = false;

        final int ARM_DOWN = -100;
        final int ARM_UP = -900;
        final int ARM_TRAVELING = -210;

        if (gamepad2.a && armPos > -1700) {
            // pressing a makes arm go back incrementally
            armPos -= 5;
            //robot.arm.setPower(.75);
            //moveArm(-30);
        } else if (gamepad2.b && armPos < 0) {
            // pressing b makes arm go forward incrementally
            armPos += 5;
            //moveArm(30);
            //robot.arm.setPower(-.75);
        } else if (gamepad2.dpad_down) {
            // puts arm and blockGripper in down position and grips block
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_OPEN);
            armPos = ARM_DOWN;
            grabbingBlock = true;
//        } else if (gamepad2.dpad_left) {
//            // puts arm and blockGripper in traveling position
//            robot.blockGripper.setPosition(0.2);
//            armPos = -210;
        } else if (gamepad2.dpad_up) {
            // puts arm and blockGripper in up position
            armPos = ARM_UP;
            power = .25;
        }

        robot.arm.setTargetPosition(armPos);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);

        if (grabbingBlock) {
            // closes blockGripper (intakes stone with gripper)
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_CLOSED);

            // moves arm to traveling position
            robot.arm.setTargetPosition(ARM_TRAVELING);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // not sure if this is needed
            robot.arm.setPower(power); // not sure if this is needed
        }
    }

    public void intakeProcessIn() {
        if (gamepad2.right_trigger > .5) {
            if (gamepad2.left_trigger > .5 && gamepad2.right_trigger > .5) {
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            } else {
                robot.leftIntake.setPower(.3);
                robot.rightIntake.setPower(-.3);
            }

        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    public void intakeProcessOut(){
        if (gamepad2.left_trigger > .5) {
            if (gamepad2.left_trigger > .5 && gamepad2.right_trigger > .5) {
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            } else {
                robot.leftIntake.setPower(-1);
                robot.rightIntake.setPower(1);
            }

        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    public void drivetrainProcess(){
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x *.75;
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
            front_left /= (max/maxSpeed);
            front_right /= (max/maxSpeed);
            rear_left /= (max/maxSpeed);
            rear_right /= (max/maxSpeed);
        }

        if(gamepad1.a){//if a strafe left
            robot.frontLeft.setPower(-1);
            robot.frontRight.setPower(1);
            robot.backLeft.setPower(1);
            robot.backRight.setPower(-1);
        } else if(gamepad1.b){//if b strafe right
            robot.frontLeft.setPower(1);
            robot.frontRight.setPower(-1);
            robot.backLeft.setPower(-1);
            robot.backRight.setPower(1);
        } else {//otherwise joysticks
            robot.frontLeft.setPower(front_left);
            robot.frontRight.setPower(front_right);
            robot.backLeft.setPower(rear_left);
            robot.backRight.setPower(rear_right);
        }
    }
}
