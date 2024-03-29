

package org.firstinspires.ftc.teamcode.testing.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//@Disabled
@Autonomous
@Disabled
public class BlueDetector extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private Servo servoTest;
    OpenCvCamera webcam;

    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static int valMidB = -1;
    private static int valLeftB = -1;
    private static int valRightB = -1;

    private static int valMidR = -1;
    private static int valLeftR = -1;
    private static int valRightR = -1;

    private static float rectHeight = 1f/8f;
    private static float rectWidth =  1f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {1f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    @Override
    public void runOpMode()
    {
        //servoTest = hardwareMap.get(Servo.class, "servoTest");
        //servoTest.setPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new SamplePipeline());

        //the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);



        //code needed for camera to display on FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();

        telemetry.addData("Values Cb Mat", valLeftB+"   "+valMidB+"   "+valRightB);
        telemetry.addData("Values Cr Mat", valLeftR+"   "+valMidR+"   "+valRightR);
        //telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (valLeftB > valMidB && valLeftB > valRightB) {
            telemetry.addData("Position", "Left Blue");
            telemetry.addData("Values Cb Mat", valLeftB+"   "+valMidB+"   "+valRightB);
            telemetry.update();
            // move to 0 degrees.
            //servoTest.setPosition(0);
            sleep(1000);
        }
        else if (valMidB > valLeftB && valMidB > valRightB) {
            telemetry.addData("Position", "Middle Blue");
            telemetry.addData("Values Cb Mat", valLeftB+"   "+valMidB+"   "+valRightB);
            telemetry.update();
            // move to 90 degrees.
            //servoTest.setPosition(0.5);
            sleep(1000);
        }

        else if (valRightB > valLeftB && valRightB > valMidB) {
            telemetry.addData("Position", "Right Blue");
            telemetry.addData("Values Cb Mat", valLeftB+"   "+valMidB+"   "+valRightB);
            telemetry.update();
            // move to 180 degrees.
            //servoTest.setPosition(1);
            sleep(1000);
        }


        telemetry.update();
        sleep(10000);
        //call movement functions

    }

    public class SamplePipeline extends OpenCvPipeline
    {
        Mat yCbCr = new Mat();
        Mat yMat = new Mat();
        Mat CbMat = new Mat();
        Mat CrMat = new Mat();
        Mat thresholdMatB = new Mat();
        Mat thresholdMatR = new Mat();
        Mat all = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCr, yMat, 0);//extracts cb channel as black and white RGB
        Core.extractChannel(yCbCr, CrMat, 1);//extracts cb channel as black and white RGB
        Core.extractChannel(yCbCr, CbMat, 2);//extracts cb channel as black and white RGB
        Imgproc.threshold(CbMat, thresholdMatB, 130, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(CrMat, thresholdMatR, 120, 255, Imgproc.THRESH_BINARY_INV);
        //any pixel with a hue value less than 102 is being set to 0 (yellow)
        //any pixel with a hue value greater than 102 is being set to 255(blue)
        //Then swaps the blue and the yellows with the binary inv line
        CbMat.copyTo(all);//copies mat object

        //get values from frame
        double[] pixMidB = CbMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMidB = (int)pixMidB[0];

        double[] pixLeftB = CbMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeftB = (int)pixLeftB[0];

        double[] pixRightB = CbMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRightB = (int)pixRightB[0];

        //get values from frame
        double[] pixMidR = CrMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMidR = (int)pixMidR[0];

        double[] pixLeftR = CrMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeftR = (int)pixLeftR[0];

        double[] pixRightR = CrMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRightR = (int)pixRightR[0];

        //get values from frame
        double[] pixMid = thresholdMatB.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMid = (int)pixMid[0];

        double[] pixLeft = thresholdMatB.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeft = (int)pixLeft[0];

        double[] pixRight = thresholdMatB.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRight = (int)pixRight[0];

        //create three points
        Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
        Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
        Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

        //draw circles on those points
        Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

        //draw 3 rectangles
        Imgproc.rectangle(//1-3
                all,
                new Point(
                        input.cols()*(leftPos[0]-rectWidth/2),
                        input.rows()*(leftPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(leftPos[0]+rectWidth/2),
                        input.rows()*(leftPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//3-5
                all,
                new Point(
                        input.cols()*(midPos[0]-rectWidth/2),
                        input.rows()*(midPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(midPos[0]+rectWidth/2),
                        input.rows()*(midPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//5-7
                all,
                new Point(
                        input.cols()*(rightPos[0]-rectWidth/2),
                        input.rows()*(rightPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(rightPos[0]+rectWidth/2),
                        input.rows()*(rightPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);


        //return input; // this is the line that declares which image is returned to the viewport (DS)
        //return CbMat;
        return all;
    }
}
}