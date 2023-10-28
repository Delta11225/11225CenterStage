package org.firstinspires.ftc.teamcode.testing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LinearSlideTest extends LinearOpMode {
   private DcMotor linearSlide;
   private Servo arm;
   @Override
   public void runOpMode() {
      linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
      linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      linearSlide.setDirection(DcMotor.Direction.REVERSE);
      linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      arm = hardwareMap.get(Servo.class,"arm" );

       // code needed for camera to display on FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        //FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();

        arm.setPosition(0);

      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.dpad_up&& linearSlide.getCurrentPosition() < 1900 ) {
            linearSlide.setPower(0.5);
         } else if (gamepad1.dpad_down && linearSlide.getCurrentPosition() > 0) {
            linearSlide.setPower(-0.5);
         } else {
            linearSlide.setPower(0.0);
         }

         if (gamepad1.a) {
            arm.setPosition(0.5);
         }
         else if (gamepad1.b) {
            arm.setPosition(0);
         }
         telemetry.addData("encoder",linearSlide.getCurrentPosition());
         telemetry.update();
      }
   }
}
