package org.firstinspires.ftc.teamcode.testing.teleop;

// import stuff

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
// @Disabled
public class ScissorLift extends LinearOpMode {
   DcMotor leftScissor;
   DcMotor rightScissor;
   @Override
   public void runOpMode() {

      leftScissor = hardwareMap.get(DcMotor.class, "left_scissor");
      rightScissor = hardwareMap.get(DcMotor.class, "right_scissor");
      waitForStart();
      while (opModeIsActive()) {
         rightScissor.setPower(-gamepad1.right_stick_y);
         leftScissor.setPower(-gamepad1.left_stick_y);
      }
      }
   }
