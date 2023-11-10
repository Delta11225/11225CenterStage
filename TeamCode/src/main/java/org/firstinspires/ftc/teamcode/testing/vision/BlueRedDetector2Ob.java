
package org.firstinspires.ftc.teamcode.testing.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
public class BlueRedDetector2Ob extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private Servo servoTest;
    OpenCvCamera webcam;

    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valRight = -1;

    private static int valMidB = -1;
    private static int valRightB = -1;

    private static int valMidR = -1;
    private static int valRightR = -1;

    private static float rectHeight = 1f/8f;
    private static float rectWidth =  1f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive


    private static float[] midPos = {1f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] rightPos = {5f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private Servo servoRight;
    private Servo servoLeft;

    @Override
    public void runOpMode()
    {
        //servoTest = hardwareMap.get(Servo.class, "servoTest");
        //servoTest.setPosition(0);

        servoRight = hardwareMap.get(Servo.class, "servo_right");
        servoLeft = hardwareMap.get(Servo.class, "servo_left");

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


        telemetry.addData("Values", valMid+"   "+valRight);

        telemetry.addData("ValuesR", valMidR+"   "+valRightR);

        telemetry.addData("ValuesB", valMidB+"   "+valRightB);

        telemetry.update();

        waitForStart();

        runtime.reset();

        if (valMidB == 255 || valMidR == 255) {
            telemetry.addData("Position", "Mid");
            telemetry.update();
            // move to 90 degrees.
            servoLeft.setPosition(0.2);
            servoRight.setPosition(0.7);
            sleep(1000);
        }

        if (valRightB == 255 || valRightR == 255) {
            telemetry.addData("Position", "Right");
            telemetry.update();
            servoLeft.setPosition(0.2);
            servoRight.setPosition(1);
            sleep(1000);
        }

        else if (valMidB != 255 && valRightR != 255 && valRightB != 255 && valMidR != 255){
            telemetry.addData("Position", "Left");
            telemetry.update();
            servoLeft.setPosition(0);
            servoRight.setPosition(0.7);
        }




        telemetry.update();
        sleep(10000);
        //call movement functions

    }

    public class SamplePipeline extends OpenCvPipeline
    {
        Mat yCbCr = new Mat();
    //    Mat yMat = new Mat();
        Mat CbMat = new Mat();
        Mat CrMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat thresholdMatCb = new Mat();
        Mat thresholdMatCr = new Mat();
        Mat all = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCr, CrMat, 1);//extracts cr channel as black and white RGB
        Core.extractChannel(yCbCr, CbMat, 2);//extracts cb channel as black and white RGB
      Imgproc.threshold(CbMat, thresholdMatCb, 150, 255, Imgproc.THRESH_BINARY);
       //anypixel with a hue value less than 150 is set to 0 (not blue)
       //any pixel with a hue value greater than 150 is set to 255 (blue)
        Imgproc.threshold(CrMat, thresholdMatCr, 150, 255, Imgproc.THRESH_BINARY);
        //any pixel with a hue value of less than 150 is set to 0 (not red)
        //any pixel with a hue value greater than 150 is se to 255 (red)
        CrMat.copyTo(all);//copies mat object

        //get values from frame
        double[] pixMidR = thresholdMatCr.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMidR = (int)pixMidR[0];

        double[] pixRightR = thresholdMatCr.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRightR = (int)pixRightR[0];

        //get values from frame
        double[] pixMidB = thresholdMatCb.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMidB = (int)pixMidB[0];

        double[] pixRightB = thresholdMatCb.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRightB = (int)pixRightB[0];

        //create three points
        Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
        Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

        //draw circles on those points
        Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

        //draw 3 rectangles
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