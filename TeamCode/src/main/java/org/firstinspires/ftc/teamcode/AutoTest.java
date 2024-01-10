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
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutonomousDeploy;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutonomousDrop;

@Autonomous
@Disabled
public class AutoTest extends LinearOpMode {
   HardwareCC robot;
   public int slideZero;
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
      slideZero = robot.linearSlide.getCurrentPosition();
      waitForStart();
      deployPixel();

   }

   public void deployPixel() {

      robot.Clamp.setPosition(clampClosedPosition);
      sleep(500);
      robot.Arm.setPosition(armHoldPosition);
      sleep(1000);
      /////Automated deploy to LOW

      robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDeploy);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setPower(1);
      while (robot.linearSlide.isBusy()) {
      }
      sleep(1500);
      robot.Arm.setPosition(armScoringPosition);
      sleep(1000);
      robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDrop);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setPower(1);
      while (robot.linearSlide.isBusy()) {
      }
      sleep(1500);
      //open clamp
      robot.Clamp.setPosition(clampOpenPosition);
      sleep(500);
      robot.linearSlide.setTargetPosition(slideZero + linearSlideAutonomousDeploy);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setPower(1);
      while (robot.linearSlide.isBusy()) {
      }
      sleep(1500);
      //return to ground
      lastSlideDown.reset();
      robot.Clamp.setPosition(clampClosedPosition);
      robot.Arm.setPosition(armHoldPosition);
      sleep(1000);
      robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.linearSlide.setTargetPosition(slideZero);
      robot.linearSlide.setPower(1);
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

}
