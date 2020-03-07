package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OmegaBot;

@TeleOp(name = "LinearTeleop", group = "prototype")
public class LinearTeleop extends LinearOpMode {
    private OmegaBot robot;
    private ElapsedTime runtime;
    double maxSpeed = 1;
    boolean intakeOn = true;
    int armPos = OmegaBot.ARM_INIT;
    int count = 0;
    boolean pickedUp = false;
    boolean armMovedBack = false;

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
            sensorPickupProcess();
            intakeProcess();
            capstoneProcess();
            foundationGrippers();

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

    public void servoProcess() {
        if (gamepad2.x) {
            robot.blockRotator.setPosition(OmegaBot.BLOCK_ROTATOR_STRAIGHT);
        } else if (gamepad2.y) {
            robot.blockRotator.setPosition(OmegaBot.BLOCK_ROTATOR_ROTATED);
        }

        if (gamepad2.right_bumper) {
            // opens block gripper
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_OPEN);
            sleep(350);

            // only resets pickedUp if the block was outtaken by the arm
            // so that the drivers can outtake with compliant wheel intake
            if (armMovedBack) {
                // moves arm back to init position
                armPos = OmegaBot.ARM_INIT;
                robot.arm.setTargetPosition(armPos);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);

                pickedUp = false;
                armMovedBack = false;
            }

        } else if (gamepad2.left_bumper) {
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_CLOSED);
        }

        if (gamepad1.x) {
            robot.sideBackGripper.setPosition(OmegaBot.SIDE_BACK_GRIPPER_CLOSED);
        } else if (gamepad1.y) {
            robot.sideBackGripper.setPosition(OmegaBot.SIDE_BACK_GRIPPER_OPEN);
        }
        telemetry.addData("Target Position: ", robot.arm.getTargetPosition());
        telemetry.addData("Actual Position: ", robot.arm.getCurrentPosition());
    }

    public void foundationGrippers() {
        if (gamepad1.left_trigger > 0.5) {
            robot.foundationGripper.setPosition(OmegaBot.FOUNDATION_GRIPPER_DOWN);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.foundationGripper.setPosition(OmegaBot.FOUNDATION_GRIPPER_UP);
        } else if (gamepad2.dpad_right) {
            robot.foundationGripper.setPosition(OmegaBot.FOUNDATION_GRIPPER_READY);
        }
    }

    public void armProcess() {
        double power = .5;
        boolean grabbingBlock = false;

        if (gamepad2.a && armPos > -1700) {
            // makes arm go back incrementally
            armPos -= 5;
        } else if (gamepad2.b && armPos < 0) {
            // makes arm go forward incrementally
            armPos += 5;
        } else if (gamepad2.dpad_down) {
            // puts arm and blockGripper in down position and grips block
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_OPEN);
            armPos = OmegaBot.ARM_DOWN;
            grabbingBlock = true;
        } else if (gamepad2.dpad_up && armPos == OmegaBot.ARM_UP && !robot.arm.isBusy()) {
            // puts arm and blockGripper in up position
            armPos = OmegaBot.ARM_UP - 300;
            power = .6;
        } else if (gamepad2.dpad_up && armPos != OmegaBot.ARM_UP - 300) {
            // puts arm and blockGripper in up position
            armPos = OmegaBot.ARM_UP;
            power = .6;
        }

        robot.arm.setTargetPosition(armPos);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(power);

        if (grabbingBlock) {
            // sleep so that blockGripper can actually open then close
            sleep(1500);

            // intake stone with gripper
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_CLOSED);

            // move arm to traveling position
            armPos = OmegaBot.ARM_TRAVELING;
            robot.arm.setTargetPosition(armPos);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(power);
        }

        // arm is moved back if it's been moved to or past the ARM UP position
        armMovedBack = armPos <= OmegaBot.ARM_UP;
    }

    public void intakeProcess() {
        if (gamepad2.left_trigger > .5) {
            // pressing both triggers stops running the intake
            if (gamepad2.left_trigger > .5 && gamepad2.right_trigger > .5) {
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            } else {
                // pressing just left trigger intakes a block
                robot.leftIntake.setPower(-1);
                robot.rightIntake.setPower(1);

                // if the block was automatically intaken using sensorPickupProcess,
                // set pickedUp to false (since outtake was just run)
                pickedUp = false;

                // while intaking, have arm at high enough position to intake
                while (gamepad2.left_trigger > .5) {
                    // move arm up enough to outtake block using compliant wheel intake
                    armPos = OmegaBot.ARM_INIT - 100;
                    robot.arm.setTargetPosition(armPos);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // be able to drive while intaking
                    drivetrainProcess();

                    // run sensorPickupProcess() if a block is close enough (has been intaken)
                    boolean isBlockIntaked = robot.sensorDistance.getDistance(DistanceUnit.CM) < 7;
                    if (isBlockIntaked) {
                        sensorPickupProcess();
                    }
                }

                // move arm back down
                armPos = OmegaBot.ARM_TRAVELING;
                robot.arm.setTargetPosition(armPos);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        } else if (gamepad2.right_trigger > .5) {
            // pressing both triggers stops running the intake
            if (gamepad2.left_trigger > .5 && gamepad2.right_trigger > .5) {
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            } else {
                // pressing just right trigger outtakes the block

                // open block gripper
                robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_OPEN);

                // run outtakes
                robot.leftIntake.setPower(.3);
                robot.rightIntake.setPower(-.3);

                sleep(500);

                // while outtaking, have arm at high enough position to outtake
                while (gamepad2.right_trigger > .5) {
                    // move arm up enough to outtake block using compliant wheel intake
                    armPos = OmegaBot.ARM_INIT - 100;
                    robot.arm.setTargetPosition(armPos);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // be able to drive while outtaking
                    drivetrainProcess();
                }

                // move arm back down
                armPos = OmegaBot.ARM_TRAVELING;
                robot.arm.setTargetPosition(armPos);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    public void sensorPickupProcess() {
        boolean isBlockIntaked = robot.sensorDistance.getDistance(DistanceUnit.CM) < 7;
        double armPower = 0.5;

        if (isBlockIntaked && !pickedUp) {
            pickedUp = true;

            // move arm down to intake block
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(armPower);
            sleep(400);

            // grip the block
            robot.blockGripper.setPosition(OmegaBot.BLOCK_GRIPPER_CLOSED);
            sleep(150);

            // move arm to traveling (init) position
            armPos = OmegaBot.ARM_TRAVELING;
            robot.arm.setTargetPosition(OmegaBot.ARM_TRAVELING);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(armPower);
       }
    }
    public void capstoneProcess() {
        if (gamepad1.left_bumper) {
            robot.capstoneRotator.setPosition(OmegaBot.CAPSTONE_ROTATOR_INIT);
        }
        if (gamepad1.right_bumper) {
            robot.capstoneRotator.setPosition(OmegaBot.CAPSTONE_ROTATOR_ROTATED);
        }
        if (gamepad1.dpad_down) {
            for(int i = 100; i > -1; i--) {
                robot.capstoneReleaser.setPosition(i/100.0);
                drivetrainProcess();
            }
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
            front_left /= (max/maxSpeed);
            front_right /= (max/maxSpeed);
            rear_left /= (max/maxSpeed);
            rear_right /= (max/maxSpeed);
        }

        if (gamepad1.a) { //if a strafe left
            robot.frontLeft.setPower(-1);
            robot.frontRight.setPower(1);
            robot.backLeft.setPower(1);
            robot.backRight.setPower(-1);
        } else if(gamepad1.b) { //if b strafe right
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
