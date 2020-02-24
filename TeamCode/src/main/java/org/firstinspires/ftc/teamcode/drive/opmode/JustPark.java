package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OmegaBot;
import org.firstinspires.ftc.teamcode.MotionMethods;

@Autonomous(group = "drive")

// pre-condition: robot is as close to skybridge as possible
public class JustPark extends LinearOpMode {
    OmegaBot robot;
    MotionMethods motionMethods;

    @Override
    public void runOpMode() throws InterruptedException {
        // edit as needed (units: inches)
        double distance = 3;

        // initialize robot and drivetrain
        robot = new OmegaBot(telemetry, hardwareMap);

        // move forward 3 inches
        motionMethods.moveMotionProfile(distance, 1);
    }
}
