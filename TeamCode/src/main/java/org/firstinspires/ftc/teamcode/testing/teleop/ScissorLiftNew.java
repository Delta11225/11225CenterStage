package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class ScissorLiftNew extends LinearOpMode {
   private DcMotor leftScissor;
   private DcMotor rightScissor;
   @Override
   public void runOpMode() {
      leftScissor = hardwareMap.get(DcMotor.class, "left_scissor");
      leftScissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftScissor.setDirection(DcMotor.Direction.REVERSE);
      leftScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightScissor = hardwareMap.get(DcMotor.class, "right_scissor");
      rightScissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightScissor.setDirection(DcMotor.Direction.REVERSE);
      rightScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.dpad_up && leftScissor.getCurrentPosition() < 40000) {
            leftScissor.setPower(1);
         } else if (gamepad1.dpad_down && rightScissor.getCurrentPosition() > 0) {
            leftScissor.setPower(-1);
         } else {
            leftScissor.setPower(0.0);
         }
         if (gamepad1.dpad_left && rightScissor.getCurrentPosition() < 40300) {
            rightScissor.setPower(1);
         } else if (gamepad1.dpad_right && rightScissor.getCurrentPosition() > 0) {
            rightScissor.setPower(-1);
         } else {
            rightScissor.setPower(0.0);
         }
         telemetry.addData("left encoder",leftScissor.getCurrentPosition());
         telemetry.addData("right encoder",rightScissor.getCurrentPosition());
         telemetry.update();
      }
   }
}
