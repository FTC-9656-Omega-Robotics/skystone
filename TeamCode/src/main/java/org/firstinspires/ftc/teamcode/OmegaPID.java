package org.firstinspires.ftc.teamcode;

/**
 * Designed to return power to DcMotors to turn accordingly to gyro
 */
public class OmegaPID{
    private double prevError = 0;
    private double dError = 0;
    private double iError = 0;
    private double pGain = 0;
    private double iGain = 0;
    private double dGain = 0;
    private double threshold = 0;
    private double diagnosticCalculatedPower = -100;

    /**
     * Initialize PID controller with constants for calibration
     * @param pGain weighted constant for position
     * @param iGain weighted constant for integral
     * @param dGain weighted constant for derivative
     * @param threshold tolerance
     */
    public OmegaPID(double pGain, double iGain, double dGain, double threshold){
        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
        this.threshold = threshold;
    }

    /**
     * Return power that is desired for the motors to turn appropriately according to sensed gyro values and PID
     * @param currentAngle current angle in degrees
     * @param desiredAngle desired angle in degrees
     * @param minPower min power robot should turn at
     * @param maxPower max power robot should turn at
     * @return
     */
    public double calculatePower(double currentAngle, double desiredAngle, double minPower, double maxPower){
        double error = desiredAngle - currentAngle;
        if(Math.abs(error) > threshold) {
            iError += error;
            dError = error - prevError;//a negative number bc current error shd be less than previous error
            prevError = error;
            double power = (error * pGain) + (dError * dGain) + (iError * iGain);
            diagnosticCalculatedPower = power;
            if (power > maxPower) {
                return maxPower;
            }
            if (power < minPower) {
                return minPower;
            }
            return power;
        }
        return 0;
    }

    public double getDiagnosticCalculatedPower() {
        return diagnosticCalculatedPower;
    }

}