package org.firstinspires.ftc.teamcode.testing.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp

public class MotorTestJally extends LinearOpMode {
   DcMotor Oubloc;
   public void runOpMode() {
      waitForStart();
      while (opModeIsActive()) {
         Oubloc = hardwareMap.get(DcMotor.class, "left_scissor");
         if (gamepad1.a) {
            Oubloc.setPower(1);
         }
      }
   }
}

