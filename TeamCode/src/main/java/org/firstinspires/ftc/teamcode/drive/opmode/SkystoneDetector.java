package org.firstinspires.ftc.teamcode.drive.opmode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    String position;
    public SkystoneDetector(){

    }
    @Override

    public final Mat processFrame(Mat input){
        input.copyTo(workingMatrix);
        if(workingMatrix.empty()){
            return input;
        }
        Imgproc.cvtColor(workingMatrix,workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(120,150,10,50);
        Mat matCenter = workingMatrix.submat(120,150,80,120);
        Mat matRight = workingMatrix.submat(120,150,150,190);

        Imgproc.rectangle(workingMatrix,new Rect(10,120,40,30),new Scalar(255,0,0));
        Imgproc.rectangle(workingMatrix,new Rect(80,120,40,30),new Scalar(255,0,0));
        Imgproc.rectangle(workingMatrix,new Rect(150,120,40,30),new Scalar(255,0,0));


        double leftTotal = Core.sumElems(matLeft).val[2]/((double)matLeft.cols()*matLeft.rows());
        double centerTotal = Core.sumElems(matCenter).val[2]/((double)matCenter.cols()*matCenter.rows());
        double rightTotal = Core.sumElems(matRight).val[2]/((double)matRight.cols()*matRight.rows());

        if(leftTotal>centerTotal){
            if(leftTotal>rightTotal){
                //left is skystone
                position = "left";
            }else{
               //right is skystone
                position = "right";
            }
        } else{
            if(centerTotal>rightTotal){
                //center is Skystone
                position = "center";
            }
            else{
                // right is skystone
                position = "right";
            }
        }

        return workingMatrix;
    }

    public String getPosition(){
        return position;
    }
}

