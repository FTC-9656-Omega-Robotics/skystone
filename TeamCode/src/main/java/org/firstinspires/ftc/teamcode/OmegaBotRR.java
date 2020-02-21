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

public class OmegaBotRR {
    // telemetry and hardwaremap come from each opmode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // DC motors
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
    final double SIDE_BACK_GRIPPER_STOWED = 0.63;
    final double SIDE_BACK_GRIPPER_OPEN = 0.41;
    final double SIDE_BACK_GRIPPER_CLOSED = 0.05;

    final double SIDE_BACK_ELBOW_UP = 0.31;
    final double SIDE_BACK_ELBOW_DOWN = 0;

    final double SIDE_FRONT_GRIPPER_STOWED = 0.35;
    final double SIDE_FRONT_GRIPPER_OPEN = 0.91;
    final double SIDE_FRONT_GRIPPER_CLOSED = 0.56;

    final double SIDE_FRONT_ELBOW_UP = 0.33; //gripper sticks out, we will need a SIDE_FRONT_ELBOW_STOWED after phone is moved
    final double SIDE_FRONT_ELBOW_DOWN = 0;

    final double CAPSTONE_HELD = 0.9;
    final double CAPSTONE_DROPPED = 0.28;

    final double BLOCK_GRIPPER_CLOSED = 0.2;
    final double BLOCK_GRIPPER_OPEN = 0.5;

    final double BLOCK_ROTATOR_STRAIGHT = 0.62;
    final double BLOCK_ROTATOR_ROTATED = 0.96;

    final double FOUNDATION_GRIPPER_UP = 0.55;
    final double FOUNDATION_GRIPPER_DOWN = 1;



    // UNKNOWN POSITIONS - check what position each number corresponds to
    // values from LinearTeleop.servoProcess()
    // block rotator positions: 0.62 [intaking/longwaysposition], 0.96 [rotated 90 degrees position]
    // block gripper positions: 0.5 [open], 0.2 [closed]
    // sideBackGripper positions: 0 [?], 0.5 [?]

    // sensors
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

    public int relativeLayoutId;
    public View relativeLayout;


    DcMotor.RunMode myRunMode = DcMotor.RunMode.RUN_USING_ENCODER;


    public OmegaBotRR(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        // Configure DcMotors with REV Expansion Hub
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");

        // Configure servos with REV Expansion Hub
        blockRotator = hardwareMap.get(Servo.class, "block_rotator");
        blockGripper = hardwareMap.get(Servo.class, "block_gripper");

        foundationGripper = hardwareMap.get(Servo.class, "foundation_gripper");

        sideBackGripper = hardwareMap.get(Servo.class, "side_back_gripper");
        sideBackElbow = hardwareMap.get(Servo.class, "side_back_elbow");

        sideFrontElbow = hardwareMap.get(Servo.class, "side_front_elbow");
        sideFrontGripper = hardwareMap.get(Servo.class, "side_front_gripper");

        capstone = hardwareMap.get(Servo.class, "capstone");

        // Configure sensors with REV Expansion Hub
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_distance_sensor");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_distance_sensor");

        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // Initialize arm
        arm.setTargetPosition(-200);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.5);

        // Initialize servos
        sideBackGripper.setPosition(SIDE_BACK_GRIPPER_STOWED);
        sideBackElbow.setPosition(SIDE_BACK_ELBOW_UP);

        sideFrontGripper.setPosition(SIDE_FRONT_GRIPPER_STOWED);
        sideFrontElbow.setPosition(SIDE_FRONT_ELBOW_UP);

        capstone.setPosition(CAPSTONE_HELD);

        // Need to get servo position constants first, then uncomment stuff below

        blockRotator.setPosition(BLOCK_ROTATOR_STRAIGHT);
        blockGripper.setPosition(BLOCK_GRIPPER_OPEN);
        foundationGripper.setPosition(FOUNDATION_GRIPPER_UP);
        //sideFrontElbow.setPosition(SIDE_FRONT_ELBOW_UP);





        /**
         * Resets the cumulative angle tracking to zero.
         */


        /**
         * Get current cumulative angle rotation from last reset.
         *
         * @return Angle in degrees. + = left, - = right.
         */


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
    } }