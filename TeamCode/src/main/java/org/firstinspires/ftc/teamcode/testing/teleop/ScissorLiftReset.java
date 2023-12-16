package org.firstinspires.ftc.teamcode.testing.teleop;

// import stuff

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class ScissorLiftReset extends LinearOpMode {
   private DcMotor leftScissor;
   private DcMotor rightScissor;
   @Override
   public void runOpMode() {
      leftScissor = hardwareMap.get(DcMotor.class, "left_scissor");
      rightScissor = hardwareMap.get(DcMotor.class, "right_scissor");
      waitForStart();
      while (opModeIsActive()) {
            leftScissor.setPower(-gamepad1.right_stick_y);
            rightScissor.setPower(-gamepad1.left_stick_y);
         }
      }
   }
