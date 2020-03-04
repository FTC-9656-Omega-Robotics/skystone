package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;
import com.qualcomm.robotcore.util.ThreadPool;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Hardware mapping for SkyStone 2019
 */

public class OmegaBotRR{
    // telemetry and hardwaremap come from each opmode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // DC motors
    public DcMotorEx arm;
    public DcMotor leftIntake;
    public DcMotor rightIntake;

    // arm position constants
    public static final int ARM_INIT = -250;
    public static final int ARM_DOWN = -100;
    public static final int ARM_UP = -1400;
    public static final int ARM_TRAVELING = -210;

    // servos
    public Servo blockRotator; // rotates block gripper on arm
    public Servo blockGripper; // opens/closes gripper on arm

    public Servo foundationGripper;

    public Servo sideBackGripper; // open/closes side back gripper
    public Servo sideBackElbow; // moves side back gripper up/down

    public Servo sideFrontGripper; // open/closes side front gripper
    public Servo sideFrontElbow; // moves side front gripper up/down

    public Servo capstoneRotator; // rotates the capstone
    public Servo capstoneReleaser; // releases the capstone

    // servo position constants
    public static final double BLOCK_ROTATOR_STRAIGHT = 0.62; // STRAIGHT is position for using blockGripper to intake
    public static final double BLOCK_ROTATOR_ROTATED = 0.96; // rotated 90 degrees from STRAIGHT

    public static final double BLOCK_GRIPPER_OPEN = 0.5;
    public static final double BLOCK_GRIPPER_CLOSED = 0.2;

    public static final double FOUNDATION_GRIPPER_UP = 0.55;
    public static final double FOUNDATION_GRIPPER_DOWN = 1;
    public static final double FOUNDATION_GRIPPER_READY = 0.85;

    public static final double SIDE_BACK_GRIPPER_OPEN = 0.41;
    public static final double SIDE_BACK_GRIPPER_CLOSED = 0;
    public static final double SIDE_BACK_GRIPPER_STOWED = 0.63; // stowed for safe travel under bridge
    public static final double SIDE_BACK_GRIPPER_READY = 0.45; // slightly closed for auto efficiency

    public static final double SIDE_BACK_ELBOW_UP = 0.31;
    public static final double SIDE_BACK_ELBOW_DOWN = 0.02;
    public static final double SIDE_BACK_ELBOW_READY = 0.15; // slightly down for auto efficiency

    public static final double SIDE_FRONT_GRIPPER_OPEN = 0.5;
    public static final double SIDE_FRONT_GRIPPER_CLOSED = 0.1;
    public static final double SIDE_FRONT_GRIPPER_STOWED = 0; // stowed for safe travel under bridge
    // public static final double SIDE_FRONT_GRIPPER_READY = [?]; // slightly closed for auto efficiency

    public static final double SIDE_FRONT_ELBOW_UP = 0.35;
    public static final double SIDE_FRONT_ELBOW_DOWN = 0.7;
    public static final double SIDE_FRONT_ELBOW_READY = 0.5; // slightly down for auto efficiency

    // TODO Add correct servo positions for the following:

    public static final double CAPSTONE_ROTATOR_INIT = 0;
    public static final double CAPSTONE_ROTATOR_ROTATED = 0.66; // rotated to release position

    public static final double CAPSTONE_RELEASER_HELD = 1;
    public static final double CAPSTONE_RELEASER_RELEASED = 0;

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

        sideFrontGripper = hardwareMap.get(Servo.class, "side_front_gripper");
        sideFrontElbow = hardwareMap.get(Servo.class, "side_front_elbow");

        capstoneRotator = hardwareMap.get(Servo.class, "capstone_rotator");
        capstoneReleaser = hardwareMap.get(Servo.class, "capstone_releaser");

        // Configure sensors with REV Expansion Hub
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_distance_sensor");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_distance_sensor");

        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // Initialize arm
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        capstoneRotator.setPosition(CAPSTONE_ROTATOR_INIT);
        capstoneReleaser.setPosition(CAPSTONE_RELEASER_HELD);

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
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Picks up a stone using the side back gripper and elbow.
     * Pre-condition: sideBackGripper is in stowed or open position
     */
    public void pickUpWithBackGripper() {
        sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_CLOSED);
        sleep(900);
        sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);
    }

    /**
     * Releases a stone using the side back gripper and elbow.
     * Pre-condition: sideBackElbow is in up position
     */
    public void releaseWithBackGripper() {
        sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_DOWN);
        sleep(500);
        sideBackGripper.setPosition(OmegaBotRR.SIDE_BACK_GRIPPER_STOWED); // stowed instead of open to be safe when traveling under bridge
        sleep(500);
        sideBackElbow.setPosition(OmegaBotRR.SIDE_BACK_ELBOW_UP);
        sleep(500);
    }

    /**
     * Picks up a stone using the side front gripper and elbow.
     * Pre-condition: sideFrontGripper is in stowed or open position
     */
    public void pickUpWithFrontGripper() {
        sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_CLOSED);
        sleep(900);
        sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);

    }

    /**
     * Releases a stone using the side front gripper and elbow.
     * Pre-condition: sideFrontElbow is in up position
     */
    public void releaseWithFrontGripper() {
        sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_DOWN);
        sleep(500);
        sideFrontGripper.setPosition(OmegaBotRR.SIDE_FRONT_GRIPPER_STOWED); // stowed instead of open to be safe when traveling under bridge
        sleep(500);
        sideFrontElbow.setPosition(OmegaBotRR.SIDE_FRONT_ELBOW_UP);
        sleep(500);
    }
}