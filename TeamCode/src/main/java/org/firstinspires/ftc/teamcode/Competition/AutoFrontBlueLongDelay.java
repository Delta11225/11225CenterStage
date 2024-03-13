package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareCC;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.utility.Constants.armCollectPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armHoldPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armScoringPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampClosedPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampOpenPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutonomousDeploy;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutonomousDrop;

//@Disabled
@Autonomous(preselectTeleOp = "CSTeleopBlue")
public class AutoFrontBlueLongDelay extends LinearOpMode {
   HardwareCC robot;
   private ElapsedTime runtime = new ElapsedTime();

   private final ElapsedTime lastSlideDown = new ElapsedTime();
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


   private static float[] midPos = {2f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
   private static float[] rightPos = {5.7f/8f+offsetX, 4f/8f+offsetY};
   //moves all rectangles right or left by amount. units are in ratio to monitor

   public boolean left = false;
   public boolean center = false;
   public boolean right = false;
   public int slideZero;


   @Override
   public void runOpMode() {

      robot = new HardwareCC(hardwareMap);

      //initialize linear slide motors
      robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.linearSlide.setDirection(DcMotor.Direction.FORWARD);
      robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      robot.linearSlide.setPower(0);
      sleep(1500);
      slideZero = robot.linearSlide.getCurrentPosition();



      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

      webcam.openCameraDevice();
      webcam.setPipeline(new AutoFrontBlueLongDelay.PropDetectionPipeline());

      //the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
      webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


      //code needed for camera to display on FTC Dashboard
      FtcDashboard dashboard = FtcDashboard.getInstance();
      telemetry = dashboard.getTelemetry();
      FtcDashboard.getInstance().startCameraStream(webcam, 10);
      telemetry.update();


      telemetry.addData("Values", valMid + "   " + valRight);

      telemetry.addData("ValuesR", valMidR + "   " + valRightR);

      telemetry.addData("ValuesB", valMidB + "   " + valRightB);

      telemetry.update();

      ////////////////////////////Trajectory Builder//////////////////////////////

      Pose2d startPose = new Pose2d(-36, 61.5, Math.toRadians(180));
      drive.setPoseEstimate(startPose);

      TrajectorySequence trajLeft = drive.trajectorySequenceBuilder(startPose)//left spike mark
              .addDisplacementMarker(()->{
                 robot.Clamp.setPosition(clampClosedPosition);
              })
              .addDisplacementMarker(()->{
                 robot.Arm.setPosition(armHoldPosition);
              })
              .waitSeconds(1)
              .lineTo(new Vector2d(-36, 45))
              .lineToLinearHeading(new Pose2d(-23.5, 31.5, Math.toRadians(225)))
              .lineToLinearHeading(new Pose2d(-36, 45, Math.toRadians(180)))
              .lineTo(new Vector2d(-36, 56.5))
              .lineTo(new Vector2d(48, 56.5))
              .addDisplacementMarker(()->{
                 robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDeploy);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(0.8);

              })
              .lineTo(new Vector2d(48, 38.5))
              .lineTo(new Vector2d(53, 38.5))


              .build();

      TrajectorySequence trajMiddle = drive.trajectorySequenceBuilder(startPose)//center spike mark
              .addDisplacementMarker(()->{
                 robot.Clamp.setPosition(clampClosedPosition);
              })
              .waitSeconds(1)
              .addDisplacementMarker(()->{
                 robot.Arm.setPosition(armHoldPosition);
              })
              .waitSeconds(1)
              .lineTo(new Vector2d(-36, 45))
              .lineTo(new Vector2d(-36, 28.5))
              .lineToLinearHeading(new Pose2d(-36, 56.5, Math.toRadians(180)))
              .lineTo(new Vector2d(48, 56.5))
              .addDisplacementMarker(()->{
                 robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDeploy);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(0.8);

              })
              .lineTo(new Vector2d(48, 33.5))
              .lineTo(new Vector2d(53, 33.5))
              .build();

      TrajectorySequence trajRight = drive.trajectorySequenceBuilder(startPose)//right spike mark
              .addDisplacementMarker(()->{
                 robot.Clamp.setPosition(clampClosedPosition);
              })
              .waitSeconds(1)
              .addDisplacementMarker(()->{
                 robot.Arm.setPosition(armHoldPosition);
              })
              .waitSeconds(1)
              .lineTo(new Vector2d(-36, 45))
              .lineToLinearHeading(new Pose2d(-42.5, 31.5, Math.toRadians(135)))
              .lineToLinearHeading(new Pose2d(-36, 45, Math.toRadians(180)))
              .lineTo(new Vector2d(-36, 57.5))
              .lineTo(new Vector2d(48, 57.5))
              .addDisplacementMarker(()->{
                 robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDeploy);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(0.8);

              })
              .lineTo(new Vector2d(48, 27.5))
              .lineTo(new Vector2d(53, 27.5))
              .build();


      waitForStart();

      runtime.reset();

////////////PROP DETECTION//////////////////////////////////////////////////////////////

      if (valMidB == 255 || valMidR == 255) {
         telemetry.addData("Position", "Mid");
         telemetry.update();
         center = true;
         // move to middle spike mark

      } else if (valRightB == 255 || valRightR == 255) {
         telemetry.addData("Position", "Right");
         telemetry.update();
         right = true;
         // move to right spike mark


      } else if (valMidB != 255 && valRightR != 255 && valRightB != 255 && valMidR != 255) {
         telemetry.addData("Position", "Left");
         telemetry.update();
         left = true;
         //move to left spike mark
      }

      webcam.stopStreaming();
      FtcDashboard.getInstance().stopCameraStream();

///////START OF ACTUAL MOVEMENT//////////////////////////////

      if (left) {
         sleep(7000);
         drive.followTrajectorySequence(trajLeft);
         deployPixel();
      } else if (right) {
         sleep(6000);
         drive.followTrajectorySequence(trajRight);
         deployPixel();
      } else {
         sleep(8000);
         drive.followTrajectorySequence(trajMiddle);
         deployPixel();
      }
   }

   public void deployPixel() {
      robot.Arm.setPosition(armScoringPosition);
      sleep(1000);
      robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDrop);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setPower(0.8);
      while (robot.linearSlide.isBusy()) {
      }
      sleep(500);
      //open clamp
      robot.Clamp.setPosition(clampOpenPosition);
      sleep(300);
      robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDeploy);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setPower(0.8);
      while (robot.linearSlide.isBusy()) {
      }
      sleep(200);
      //return to ground
      lastSlideDown.reset();
      robot.Clamp.setPosition(clampClosedPosition);
      robot.Arm.setPosition(armHoldPosition);
      sleep(1000);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setTargetPosition(slideZero);
      robot.linearSlide.setPower(0.8);
      while (robot.linearSlide.isBusy() && (lastSlideDown.seconds() < 3)) {
         telemetry.addData("LinearSlideEncoder", robot.linearSlide.getCurrentPosition());
         telemetry.addLine("Stuck in loop");
         telemetry.update();
      }
      telemetry.addData("LinearSlideEncoder", robot.linearSlide.getCurrentPosition());
      telemetry.update();
      robot.linearSlide.setPower(0);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.Arm.setPosition(armCollectPosition);
      robot.Clamp.setPosition(clampOpenPosition);
   }


   public class PropDetectionPipeline extends OpenCvPipeline
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
         //  Core.extractChannel(yCbCr, yMat, 0);//extracts cb channel as black and white RGB
         Core.extractChannel(yCbCr, CrMat, 1);//extracts cb channel as black and white RGB
         Core.extractChannel(yCbCr, CbMat, 2);//extracts cb channel as black and white RGB
         Imgproc.threshold(CbMat, thresholdMatCb, 150, 255, Imgproc.THRESH_BINARY);
         Imgproc.threshold(CrMat, thresholdMatCr, 150, 255, Imgproc.THRESH_BINARY);
         //any pixel with a hue value less than 102 is being set to 0 (yellow)
         //any pixel with a hue value greater than 102 is being set to 255(blue)
         //Then swaps the blue and the yellows with the binary inv line
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

