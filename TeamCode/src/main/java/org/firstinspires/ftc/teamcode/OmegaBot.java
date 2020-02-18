package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware mapping for SkyStone 2019
 */

public class OmegaBot {
    //telemetry an hardwaremap come from each opmode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    //DC motors we want
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx arm;
    public DcMotor extension;
    public DcMotor leftIntake;
    public DcMotor rightIntake;

    //servos we want
    public Servo pivot;
    public Servo blockGripper;
    public Servo centerGripper;
    public Servo cap;//0.41 open, 0.05 closed, this is actually elbow gripper
    public Servo rightGripper;//0.33 up, 0 down
    public Servo elbowGripper;//this is actually capstone, its just that i configged wrong
    public Servo frontElbow;
    public Servo frontWrist;//.91 open, .56 closed
    public Servo capstone;//.9 holding the capstone, .28 dropping the capstone
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public int relativeLayoutId;
    public View relativeLayout;


    DcMotor.RunMode myRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public OmegaDriveTrain drivetrain;

    //3.937-inch diameter wheels, 1 wheel rotations per 1 motor rotation; all Yellow Jacket 19.2:1 motors for wheels (537.6 ticks per rev for 1:1); 27 inch turning diameter
    final double ticksPerInch = (537.6 / 1.0) / (3.937 * Math.PI);
    final double ticksPerDegree = ticksPerInch * 27 * Math.PI / 360.0 * (2.0 / 3); //2.0 / 3 is random scale factor
    final double turnTolerance = 2; //2 degrees error tolerance
    final double driveTolerance = 8;
    final double turnTimeLimit = 2.5;
    final double driveTimeLimitPer1Foot = 0.80; //1.5 sec per 12 inches
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;//gyro
    public OmegaPID turnPID;
    public OmegaPID drivePID;
    double globalAngle, power = .30, correction;

    double MOVE_CORRECTION_ADDENDUM = 0;

    OmegaBot(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        //extension = hardwareMap.get(DcMotor.class, "extension");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");


        pivot = hardwareMap.get(Servo.class, "pivot");
        blockGripper = hardwareMap.get(Servo.class, "block_gripper");
        centerGripper = hardwareMap.get(Servo.class, "center_gripper");
        cap = hardwareMap.get(Servo.class, "cap");
        rightGripper = hardwareMap.get(Servo.class, "left_gripper");
        frontElbow = hardwareMap.get(Servo.class, "front_elbow");
        frontWrist = hardwareMap.get(Servo.class, "front_wrist");
        capstone = hardwareMap.get(Servo.class, "capstone");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_distance_sensor");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_distance_sensor");
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        elbowGripper = hardwareMap.get(Servo.class, "capstone");//port number 5
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu1".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drivetrain = new OmegaDriveTrain(frontLeft, frontRight, backLeft, backRight);
        drivetrain.setRunMode(myRunMode);
        turnPID = new OmegaPID(0.25, 0, 0.36, turnTolerance); //0.015, 0.00008, 0.05 work for robotSpeed = 0.6. now tuning for 1.0
        drivePID = new OmegaPID(0.45, 0, 0.395, driveTolerance);//.25, .0001, .08 has some jitters
    }//.25,.00008,.5

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

//    /**
//     * Get the real heading {0, 360}
//     *
//     * @return the heading of the robot {0, 360}
//     */
//    public double getAngleReadable() {
//        double a = getAngle() % 360;
//        if (a < 0) {
//            a = 360 + a;
//        }
//        return a;
//    }
//
//    /**
//     * See if we are moving in a straight line and if not return a power correction value.
//     *
//     * @return Power adjustment, + is adjust left - is adjust right.
//     */
//    public double checkDirection() {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction = correction * gain;
//
//        return correction;
//    }

    public double getMOVE_CORRECTION_ADDENDUM() {
        return MOVE_CORRECTION_ADDENDUM;
    }


    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getTurnTolerance() {
        return turnTolerance;
    }
}
