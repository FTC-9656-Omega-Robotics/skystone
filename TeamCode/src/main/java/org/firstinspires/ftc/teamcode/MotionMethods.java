package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionMethods {
    OmegaBot robot;
    Telemetry telemetry;
    LinearOpMode opMode;

    public MotionMethods(OmegaBot robot, Telemetry telemetry, LinearOpMode opMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public void moveMotionProfile(double inches, double power){//power is between 0 and 1
        if(inches == 0){
            return;
        }
        double maxVel = 312 * 3.937 * Math.PI / 60000; // 312 is the rotations per minute, 3.937 * pi is the inches per rotation (based on wheel circumference), 60000 is the number of milliseconds in a minute
        double macAcc = maxVel / 1600; //1300 is the number of milliseconds it takes to accelerate to full speed
        MotionProfileGenerator generator = new MotionProfileGenerator(maxVel * power, macAcc);//multiply by power cuz its a number between 0 and 1 so it scales
        double[] motionProfile = generator.generateProfile(inches);
        double[] distanceProfile = generator.generateDistanceProfile(motionProfile);
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runtime.reset();
        double heading = robot.getAngle();
        while(runtime.milliseconds() < motionProfile.length && opMode.opModeIsActive()){
            int ms = (int) runtime.milliseconds();
            if (ms < motionProfile.length) {
                double adjust = 0.04 * (robot.getAngle()-heading);
                robot.frontLeft.setPower(motionProfile[ms] / maxVel + adjust);
                robot.backLeft.setPower(motionProfile[ms] / maxVel + adjust);
                robot.frontRight.setPower(motionProfile[ms] / maxVel - adjust);
                robot.backRight.setPower(motionProfile[ms] / maxVel - adjust);
                //robot.drivetrain.setVelocity(motionProfile[(int) runtime.milliseconds()] / maxVel);//TODO: use the distance profile + encoders to pid up in dis bicth
            }



        }
        robot.drivetrain.setVelocity(0);
    }

    public void moveMotionProfileZeroDegrees(double inches, double power){//power is between 0 and 1
        if(inches == 0){
            return;
        }
        double maxVel = 312 * 3.937 * Math.PI / 60000; // 312 is the rotations per minute, 3.937 * pi is the inches per rotation (based on wheel circumference), 60000 is the number of milliseconds in a minute
        double macAcc = maxVel / 2000; //1300 is the number of milliseconds it takes to accelerate to full speed
        MotionProfileGenerator generator = new MotionProfileGenerator(maxVel * power, macAcc);//multiply by power cuz its a number between 0 and 1 so it scales
        double[] motionProfile = generator.generateProfile(inches);
        double[] distanceProfile = generator.generateDistanceProfile(motionProfile);
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runtime.reset();
        double heading = 0;
        while(runtime.milliseconds() < motionProfile.length && opMode.opModeIsActive()){
            int ms = (int) runtime.milliseconds();
            if (ms < motionProfile.length) {
                double adjust = 0.04 * (robot.getAngle()-heading);
                robot.frontLeft.setPower(motionProfile[ms] / maxVel + adjust);
                robot.backLeft.setPower(motionProfile[ms] / maxVel + adjust);
                robot.frontRight.setPower(motionProfile[ms] / maxVel - adjust);
                robot.backRight.setPower(motionProfile[ms] / maxVel - adjust);
                telemetry.addData("gyro pos", robot.getAngle());
                telemetry.update();
                //robot.drivetrain.setVelocity(motionProfile[(int) runtime.milliseconds()] / maxVel);//TODO: use the distance profile + encoders to pid up in dis bicth
            }



        }
        robot.drivetrain.setVelocity(0);
    }

    public void moveMotionProfileReverse(double inches, double power){//power is between 0 and 1
        if(inches == 0){
            return;
        }
        double maxVel = 312 * 3.937 * Math.PI / 60000; // 312 is the rotations per minute, 3.937 * pi is the inches per rotation (based on wheel circumference), 60000 is the number of milliseconds in a minute
        double macAcc = maxVel / 2000; //1300 is the number of milliseconds it takes to accelerate to full speed
        MotionProfileGenerator generator = new MotionProfileGenerator(maxVel * power, macAcc);//multiply by power cuz its a number between 0 and 1 so it scales
        double[] motionProfile = generator.generateProfile(inches);
        double[] distanceProfile = generator.generateDistanceProfile(motionProfile);
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runtime.reset();
        double heading = robot.getAngle();
        while(runtime.milliseconds() < motionProfile.length && opMode.opModeIsActive()){
            int ms = (int) runtime.milliseconds();
            if (ms < motionProfile.length) {
                double adjust = 0.04 * (robot.getAngle()-heading);
                robot.frontLeft.setPower(motionProfile[ms] / maxVel - adjust);
                robot.backLeft.setPower(motionProfile[ms] / maxVel - adjust);
                robot.frontRight.setPower(motionProfile[ms] / maxVel + adjust);
                robot.backRight.setPower(motionProfile[ms] / maxVel + adjust);
                //robot.drivetrain.setVelocity(motionProfile[(int) runtime.milliseconds()] / maxVel);//TODO: use the distance profile + encoders to pid up in dis bicth
            }



        }
        robot.drivetrain.setVelocity(0);
    }

    public void movePID(double inches, double velocity) {
        DcMotor.RunMode originalMode = robot.frontLeft.getMode(); //Assume that all wheels have the same runmode
        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double target = robot.ticksPerInch * inches;
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int count = 0;
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive() && runtime.seconds() < robot.driveTimeLimitPer1Foot * inches / 12.0) {
            robot.drivetrain.setVelocity(robot.drivePID.calculatePower(robot.drivetrain.getAvgEncoderValueOfFrontWheels(), target, -velocity, velocity));
            telemetry.addData("Count", count);
            telemetry.update();
        }
        robot.drivetrain.setVelocity(0);
        robot.drivetrain.setRunMode(originalMode);
    }

    /**
     * This method makes the robot turn counterclockwise based on gyro values and PID
     * Velocity is always positive. Set neg degrees for clockwise turn
     * pwr in setPower(pwr) is a fraction [-1.0, 1.0] of 12V
     *
     * @param degrees  desired angle in deg
     * @param velocity max velocity
     */
    public void turnUsingPIDVoltage(double degrees, double velocity) {
        DcMotor.RunMode original = robot.frontLeft.getMode(); //assume all drive motors r the same runmode
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double max = 12.0 * velocity;
        double targetHeading = robot.getAngle() + degrees;
        double timeLimit = 0.01 * Math.abs(robot.getAngle() - degrees);
        int count = 0;
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive() && runtime.seconds() < timeLimit) {
            velocity = (robot.turnPID.calculatePower(robot.getAngle(), targetHeading, -max, max) / 12.0); //turnPID.calculatePower() used here will return a voltage
            telemetry.addData("Count", count);
            telemetry.addData("Calculated velocity [-1.0, 1/0]", robot.turnPID.getDiagnosticCalculatedPower() / 12.0);
            telemetry.addData("PID power [-1.0, 1.0]", velocity);
            telemetry.addData("Turning now", count);
            telemetry.update();
            robot.frontLeft.setPower(-velocity);
            robot.backLeft.setPower(-velocity);
            robot.frontRight.setPower(velocity);
            robot.backRight.setPower(velocity);
            count++;
        }
        robot.drivetrain.setVelocity(0);
        robot.drivetrain.setRunMode(original);
    }

    public void turnUsingPIDVoltageFieldCentric(double degrees, double velocity) {
        DcMotor.RunMode original = robot.frontLeft.getMode(); //assume all drive motors r the same runmode
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double max = 12.0 * velocity;
        double targetHeading = degrees;
        int count = 0;
        ElapsedTime runtime = new ElapsedTime();
        double timeLimit = 0.05 * Math.abs(robot.getAngle() - degrees);
        if(Math.abs(robot.getAngle() - degrees) > 45) timeLimit = 0.012 * Math.abs(robot.getAngle() - degrees);
        if(Math.abs(robot.getAngle() - degrees) > 30) timeLimit = 0.02 * Math.abs(robot.getAngle() - degrees);
        while (opMode.opModeIsActive() && runtime.seconds() < timeLimit) {
            velocity = (robot.turnPID.calculatePower(robot.getAngle(), targetHeading, -max, max) / 12.0); //turnPID.calculatePower() used here will return a voltage
            telemetry.addData("Count", count);
            telemetry.addData("Calculated velocity [-1.0, 1/0]", robot.turnPID.getDiagnosticCalculatedPower() / 12.0);
            telemetry.addData("PID power [-1.0, 1.0]", velocity);
            telemetry.update();
            robot.frontLeft.setPower(-velocity);
            robot.backLeft.setPower(-velocity);
            robot.frontRight.setPower(velocity);
            robot.backRight.setPower(velocity);
            count++;
        }
        robot.drivetrain.setVelocity(0);
        robot.drivetrain.setRunMode(original);
    }

    public void strafe(double heading, double time, double velocity){
        double moveGain = .02;
        double turnGain = .08;
        double right = Math.cos(Math.toRadians(heading));
        double forward = Math.sin(Math.toRadians(heading));
        double maxVel = 312 * 3.937 * Math.PI / 60;
        telemetry.addData("heading", heading);
        telemetry.update();
        double robotHeading = robot.getAngle();
        int[] encoderCounts = {robot.frontLeft.getCurrentPosition(),robot.frontRight.getCurrentPosition(),robot.backLeft.getCurrentPosition(),robot.backRight.getCurrentPosition()};
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double currTime = runtime.milliseconds();
        while (opMode.opModeIsActive() && runtime.milliseconds() < time * 1000){
            double timeChange = runtime.milliseconds() - currTime;
            currTime = runtime.milliseconds();
            double clockwise =  robot.getAngle() - robotHeading;
            clockwise *= turnGain;
            /**
             double temp = forward * Math.cos(Math.toRadians(robotHeading)) - right * Math.sin(Math.toRadians(robotHeading));
             right = forward * Math.sin(Math.toRadians(robotHeading)) + right * Math.cos(Math.toRadians(robotHeading));
             forward = temp * moveGain * distance;
             right = right * moveGain * distance;
             **/

            double front_left = forward + clockwise + right;
            double front_right = forward - clockwise -right;
            double rear_left = forward + clockwise - right;
            double rear_right = forward - clockwise + right;

            double max = Math.abs(front_left);
            if(Math.abs(front_right) > max) max = Math.abs(front_right);
            if(Math.abs(rear_left) > max) max = Math.abs(rear_left);
            if(Math.abs(rear_right) > max) max = Math.abs(rear_right);

            if(max>velocity){
                front_left /= max;
                front_left *= velocity;
                front_right /= max;
                front_right *= velocity;
                rear_left /= max;
                rear_left *= velocity;
                rear_right /= max;
                rear_right *= velocity;
            }

            robot.frontLeft.setPower(front_left);
            robot.frontRight.setPower(front_right);
            robot.backLeft.setPower(rear_left);
            robot.backRight.setPower(rear_right);
        }
        robot.drivetrain.setVelocity(0);
    }

    public void strafeLeft(double inches, double power){
        double ticksPerInch = 537.6/(3.937 * Math.PI);
        inches *= 1.25;
        robot.frontLeft.setTargetPosition((int) (robot.frontLeft.getCurrentPosition() - inches * ticksPerInch));
        robot.frontRight.setTargetPosition((int) (robot.frontRight.getCurrentPosition() + inches * ticksPerInch));
        robot.backLeft.setTargetPosition((int) (robot.backLeft.getCurrentPosition() + inches * ticksPerInch));
        robot.backRight.setTargetPosition((int) (robot.backRight.getCurrentPosition() - inches * ticksPerInch));
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drivetrain.setVelocity(power);
        while(robot.drivetrain.isPositioning());
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnTheCoolVivaWayFieldCentric(double degrees, double velocity) {
        DcMotor.RunMode original = robot.frontLeft.getMode(); //assume all drive motors r the same runmode
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double max = 12.0 * velocity;
        double targetHeading = degrees;
        int count = 0;
        ElapsedTime runtime = new ElapsedTime();
        double timeLimit = 0.05 * Math.abs(robot.getAngle() - degrees);
        if(Math.abs(robot.getAngle() - degrees) > 45) timeLimit = 0.012 * Math.abs(robot.getAngle() - degrees);
        while (opMode.opModeIsActive() && runtime.seconds() < timeLimit) {
            velocity = (robot.turnPID.calculatePower(robot.getAngle(), targetHeading, -max, max) / 12.0); //turnPID.calculatePower() used here will return a voltage
            telemetry.addData("Count", count);
            telemetry.addData("Calculated velocity [-1.0, 1/0]", robot.turnPID.getDiagnosticCalculatedPower() / 12.0);
            telemetry.addData("PID power [-1.0, 1.0]", velocity);
            telemetry.update();
            robot.frontLeft.setPower(-velocity);
            robot.backLeft.setPower(-velocity);
            //robot.frontRight.setPower(velocity/30);
            //robot.backRight.setPower(velocity/30);
            count++;
        }
        robot.drivetrain.setVelocity(0);
        robot.drivetrain.setRunMode(original);
    }

    public void turnTheCoolVivaWayFieldCentric2(double degrees, double velocity) {
        DcMotor.RunMode original = robot.frontLeft.getMode(); //assume all drive motors r the same runmode
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double max = 12.0 * velocity;
        double targetHeading = degrees;
        int count = 0;
        ElapsedTime runtime = new ElapsedTime();
        double timeLimit = 0.05 * Math.abs(robot.getAngle() - degrees);
        if(Math.abs(robot.getAngle() - degrees) > 45) timeLimit = 0.012 * Math.abs(robot.getAngle() - degrees);
        while (opMode.opModeIsActive() && runtime.seconds() < timeLimit) {
            velocity = (robot.turnPID.calculatePower(robot.getAngle(), targetHeading, -max, max) / 12.0); //turnPID.calculatePower() used here will return a voltage
            telemetry.addData("Count", count);
            telemetry.addData("Calculated velocity [-1.0, 1/0]", robot.turnPID.getDiagnosticCalculatedPower() / 12.0);
            telemetry.addData("PID power [-1.0, 1.0]", velocity);
            telemetry.update();
            robot.frontRight.setPower(velocity);
            robot.backRight.setPower(velocity);
            //robot.frontRight.setPower(velocity/30);
            //robot.backRight.setPower(velocity/30);
            count++;
        }
        robot.drivetrain.setVelocity(0);
        robot.drivetrain.setRunMode(original);
    }
}
