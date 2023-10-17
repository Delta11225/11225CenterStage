package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LinearSlideTeleop extends LinearOpMode {

   private DcMotor linearSlide;
   private Servo Arm;
   private Servo Collector;

   @Override
   public void runOpMode() {

      linearSlide = hardwareMap.get(DcMotor.class, "slide");
      Arm = hardwareMap.get(Servo.class, "arm");
      Collector = hardwareMap.get(Servo.class, "collector");
      linearSlide.setDirection(DcMotor.Direction.REVERSE);
      telemetry.addData("Status", "Initialized");
      telemetry.update();

      waitForStart();

      // NEED TO INCORPORATE ENCODER MAX + MIN VALUES
      // linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      while (opModeIsActive()) {
         linearSlide.setPower(-this.gamepad2.right_stick_y);
     // Need to test and set arm and collector position values once we can test different values
         if (gamepad2.x) {
            Arm.setPosition(0);
            // Collector.setPosition(0);
         }
         if (gamepad2.a) {
            Arm.setPosition(0);
            // Collector.setPosition(0);
         }
         telemetry.addData("status", "running");
         telemetry.update();
      }
   }
}


