package org.firstinspires.ftc.teamcode;

public class MotionProfileGenerator {
    public double maxVel;
    public double maxAcc;
    private double distThreshold;
    private int accTime;

    /**
     * constructor
     * @param maxVel needs to be in units/millisecond
     * @param maxAcc needs to be in units/millisecond
     */
    public MotionProfileGenerator(double maxVel, double maxAcc){
        this.maxAcc = maxAcc;
        this.maxVel = maxVel;
        distThreshold = maxVel * maxVel / maxAcc;
        accTime = (int) (2*distThreshold/maxVel);
    }

    public double[] generateProfile(double distance){
        double[] motionProfile;
        if(distance > distThreshold){
            double maxVelTime = ((distance - distThreshold)/maxVel);
            int runTime = (int) (maxVelTime + 2 * accTime);
            motionProfile = new double[runTime];
            for(int ms = 0; ms < runTime; ms++){
                if(ms < accTime){
                    motionProfile[ms] = maxAcc * ms;
                }else if(ms < runTime - accTime) {
                    motionProfile[ms] = maxVel;
                }else if(ms < runTime){
                    motionProfile[ms] = maxVel - (ms - (runTime - accTime - maxVelTime)*maxAcc);
                }
            }
        }else{
            int time = (int) (Math.sqrt(distance/maxAcc) + .5);
            motionProfile = new double[time * 2];
            for(int ms = 0; ms < time * 2; ms++){
                if(ms <= time){
                    motionProfile[ms] = maxAcc * ms;
                }else{
                    motionProfile[ms] = maxAcc * (2 * time - ms);
                }
            }
        }
        return motionProfile;
    }

    public double[] generateDistanceProfile(double[] motionProfile){
        double[] distanceProfile = new double[motionProfile.length];
        int prev = 0;
        for(int i = 0; i < motionProfile.length; i++){
            prev += motionProfile[i];
            distanceProfile[i] = prev;
        }
        return distanceProfile;
    }
}
