package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareCC;

import static org.firstinspires.ftc.teamcode.utility.Constants.armCollectPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armHoldPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armScoringPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampClosedPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampOpenPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployLow;
@Disabled
@Autonomous
public class AutoGlitch extends LinearOpMode {
   HardwareCC robot;
   private ElapsedTime runtime = new ElapsedTime();

   private final ElapsedTime lastSlideDown = new ElapsedTime();
   @Override
   public void runOpMode() throws InterruptedException {
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
      robot = new HardwareCC(hardwareMap);

      //initialize linear slide motors
      robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.linearSlide.setDirection(DcMotor.Direction.FORWARD);
      robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      robot.linearSlide.setPower(0);

      Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));
      drive.setPoseEstimate(startPose);


      TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)//center spike mark
              .addDisplacementMarker(()->{
                 robot.Clamp.setPosition(clampClosedPosition);
              })
              .waitSeconds(0.1)
              .addDisplacementMarker(()->{
                 robot.Arm.setPosition(armHoldPosition);
              })
              .waitSeconds(0.5)
              .strafeLeft(30)
              .build();


      robot.Clamp.setPosition(clampClosedPosition);


      waitForStart();
      drive.followTrajectorySequence(traj1);
   }

}
