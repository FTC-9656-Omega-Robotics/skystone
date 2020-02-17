package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OmegaDriveTrain {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx[] motors = new DcMotorEx[4];

    public OmegaDriveTrain(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.motors[0] = frontLeft;
        this.motors[1] = frontRight;
        this.motors[2] = backLeft;
        this.motors[3] = backRight;
    }

    /**
     * Set the number of ticks for the drivetrain to go forward
     *
     * @param ticks - number of ticks
     */
    public void setTargetPosition(double ticks) {
        int distance = (int) (ticks + 0.5);
        frontLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
    }

    public int getAvgEncoderValueOfFrontWheels() {
        return (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 2;
    }

    public int getAvgEncoderValueOfBackWheels() {
        return (backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
    }

    public int getAvgEncoderValueOfLeftWheels() {
        return (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2;
    }

    public int getAvgEncoderValueOfRightWheels() {
        return (frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
    }

    /**
     * Set all motors to a runmode
     *
     * @param - the run mode
     */
    public void setRunMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void reverseDirection(){
        for(int i = 0; i < 4; i++){
            if(motors[i].getDirection() == DcMotor.Direction.FORWARD){
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            }
            else{
                motors[i].setDirection(DcMotor.Direction.FORWARD);
            }
        }
    }

    /**
     * Set the velocity of all the wheels
     *
     * @param velocity - a number between -1.0 and 1.0
     */
    public void setVelocity(double velocity) {
        frontLeft.setPower(velocity);
        frontRight.setPower(velocity);
        backLeft.setPower(velocity);
        backRight.setPower(velocity);
    }

    public void setVelocityPID(double p, double i, double d, double f){
        for(DcMotorEx motor : motors){
            motor.setVelocityPIDFCoefficients(p,i,d,f);
        }
    }

    /**
     * Use as loop condition to wait for drivetrain to finish positioning before moving onto the rest of the code
     */
    public boolean isPositioning() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }
}
