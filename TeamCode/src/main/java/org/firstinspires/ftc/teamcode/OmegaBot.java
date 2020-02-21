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
    // telemetry an hardwaremap come from each opmode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // DC motors
    // wheels of drivetrain
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx arm;

    public DcMotor leftIntake;
    public DcMotor rightIntake;

    // servos
    public Servo blockRotator; // rotates block gripper on arm
    public Servo blockGripper; // opens/closes gripper on arm

    public Servo foundationGripper;

    public Servo sideBackGripper; // open/closes side back gripper
    public Servo sideBackElbow; // moves side back gripper up/down

    public Servo sideFrontGripper; // open/closes side front gripper
    public Servo sideFrontElbow; // moves side front gripper up/down

    public Servo capstone;

    // servo position constants
    static final double BLOCK_ROTATOR_STRAIGHT = 0.62; // STRAIGHT is position for using blockGripper to intake
    static final double BLOCK_ROTATOR_ROTATED = 0.96; // rotated 90 degrees from STRAIGHT

    static final double BLOCK_GRIPPER_OPEN = 0.5;
    static final double BLOCK_GRIPPER_CLOSED = 0.2;

    static final double FOUNDATION_GRIPPER_UP = 0.55;
    static final double FOUNDATION_GRIPPER_DOWN = 1;

    static final double SIDE_BACK_GRIPPER_OPEN = 0.41;
    static final double SIDE_BACK_GRIPPER_CLOSED = 0.05;
    static final double SIDE_BACK_GRIPPER_STOWED = 0.63; // stowed for robot inspection

    static final double SIDE_BACK_ELBOW_UP = 0.31;
    static final double SIDE_BACK_ELBOW_DOWN = 0;

    static final double SIDE_FRONT_GRIPPER_OPEN = 0.91;
    static final double SIDE_FRONT_GRIPPER_CLOSED = 0.56;
    static final double SIDE_FRONT_GRIPPER_STOWED = 0.35; // stowed for robot inspection

    static final double SIDE_FRONT_ELBOW_UP = 0.33;
    static final double SIDE_FRONT_ELBOW_DOWN = 0;
    //gripper sticks out, we will need a SIDE_FRONT_ELBOW_STOWED after phone is moved
    // final double SIDE_FRONT_ELBOW_STOWED = [some value]; // stowed for robot inspection

    final double CAPSTONE_HELD = 0.9;
    final double CAPSTONE_DROPPED = 0.28;

    // sensors
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

    public int relativeLayoutId;
    public View relativeLayout;


    DcMotor.RunMode myRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public OmegaDriveTrain drivetrain;

    //3.937-inch diameter wheels, 1 wheel rotations per 1 motor rotation; all Yellow Jacket 19.2:1 motors for wheels (537.6 ticks per rev for 1:1); 27 inch turning diameter
    final double TICKS_PER_INCH = (537.6 / 1.0) / (3.937 * Math.PI);
    final double TICKS_PER_DEGREE = TICKS_PER_INCH * 27 * Math.PI / 360.0 * (2.0 / 3); //2.0 / 3 is random scale factor
    final double TURN_TOLERANCE = 2; //2 degrees error tolerance
    final double DRIVE_TOLERANCE = 8;
    final double TURN_TIME_LIMIT = 2.5;
    final double DRIVE_TIME_LIMIT_PER_1_FOOT = 0.80; //1.5 sec per 12 inches
    Orientation lastAngles = new Orientation();
    BNO055IMU imu;//gyro
    public OmegaPID turnPID;
    public OmegaPID drivePID;
    double globalAngle, power = .30, correction;

    double MOVE_CORRECTION_ADDENDUM = 0;

    public OmegaBot(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        // Configure DcMotors with REV Expansion Hub
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");

        // Configure servos with REV Expansion Hub
        blockRotator = hardwareMap.get(Servo.class, "block_rotator");
        blockGripper = hardwareMap.get(Servo.class, "block_gripper");

        foundationGripper = hardwareMap.get(Servo.class, "foundation_gripper");

        sideBackGripper = hardwareMap.get(Servo.class, "side_back_gripper");
        sideBackElbow = hardwareMap.get(Servo.class, "side_back_elbow");

        sideFrontGripper = hardwareMap.get(Servo.class, "side_front_gripper");
        sideFrontElbow = hardwareMap.get(Servo.class, "side_front_elbow");

        capstone = hardwareMap.get(Servo.class, "capstone");

        // Configure sensors with REV Expansion Hub
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");

        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

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

        // Initialize drivetrain
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

        // Initialize arm
        arm.setTargetPosition(-200);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.5);

        // Initialize servos
        blockRotator.setPosition(BLOCK_ROTATOR_STRAIGHT);
        blockGripper.setPosition(BLOCK_GRIPPER_OPEN);

        foundationGripper.setPosition(FOUNDATION_GRIPPER_UP);

        sideBackGripper.setPosition(SIDE_BACK_GRIPPER_STOWED);
        sideBackElbow.setPosition(SIDE_BACK_ELBOW_UP);

        sideFrontGripper.setPosition(SIDE_FRONT_GRIPPER_STOWED);
        sideFrontElbow.setPosition(SIDE_FRONT_ELBOW_UP);

        capstone.setPosition(CAPSTONE_HELD);

        // Set up PID for drivetrain
        drivetrain = new OmegaDriveTrain(frontLeft, frontRight, backLeft, backRight);
        drivetrain.setRunMode(myRunMode);
        turnPID = new OmegaPID(0.25, 0, 0.36, TURN_TOLERANCE); //0.015, 0.00008, 0.05 work for robotSpeed = 0.6. now tuning for 1.0
        drivePID = new OmegaPID(0.45, 0, 0.395, DRIVE_TOLERANCE);//.25, .0001, .08 has some jitters
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
        return TICKS_PER_INCH;
    }

    public double getTurnTolerance() {
        return TURN_TOLERANCE;
    }
}
