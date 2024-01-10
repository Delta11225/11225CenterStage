package org.firstinspires.ftc.teamcode.testing.teleop;

// import stuff

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class ScissorLift extends LinearOpMode {
   DcMotor scissorLift;
   @Override
   public void runOpMode() {
      scissorLift = hardwareMap.get(DcMotor.class, "scissorLift");
      waitForStart();
      while (opModeIsActive()) {
            scissorLift.setPower(-gamepad1.right_stick_y);
         }
      }
   }
