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
    //telemetry an hardwaremap come from each opmode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    //DC motors we want
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


    OmegaBotRR(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;


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


        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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