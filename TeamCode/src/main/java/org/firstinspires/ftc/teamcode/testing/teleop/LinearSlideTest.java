package org.firstinspires.ftc.teamcode.testing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.utility.Constants.armCollectPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armHoldPosition;

@TeleOp
@Disabled
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

        arm.setPosition(armHoldPosition);

      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.dpad_up&& linearSlide.getCurrentPosition() < 4100 ) {
            linearSlide.setPower(1);
         } else if (gamepad1.dpad_down && linearSlide.getCurrentPosition() > 0) {
            linearSlide.setPower(-1);
         } else {
            linearSlide.setPower(0.0);
         }

         if (gamepad1.a) {
            arm.setPosition(armCollectPosition);
         }
         else if (gamepad1.b) {
            arm.setPosition(armHoldPosition);
         }
         telemetry.addData("encoder",linearSlide.getCurrentPosition());
         telemetry.update();
      }
   }
}
