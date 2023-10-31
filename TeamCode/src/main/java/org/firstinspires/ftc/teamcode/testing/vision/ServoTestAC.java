package org.firstinspires.ftc.teamcode.testing.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTestAC extends LinearOpMode {
   private Servo servoLeft;
   private Servo servoRight;

   @Override
   public void runOpMode() {

      //code needed for camera to display on FTC Dashboard
      FtcDashboard dashboard = FtcDashboard.getInstance();
      telemetry = dashboard.getTelemetry();
      // FtcDashboard.getInstance().startCameraStream(webcam, 10);
      telemetry.update();

      servoLeft = hardwareMap.get(Servo.class, "servo_left");
      servoRight = hardwareMap.get(Servo.class, "servo_right");
      telemetry.addData("Status", "Initialized");
      telemetry.update();
      waitForStart();

      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.x) {
            servoLeft.setPosition(0);
            servoRight.setPosition(0.7);
         }
         if (gamepad1.a) {
            servoLeft.setPosition(0.2);
            servoRight.setPosition(0.7);
         }
         if (gamepad1.b) {
            servoLeft.setPosition(0.2);
            servoRight.setPosition(1);
         }
         telemetry.addData("Servo Position", servoLeft.getPosition());
         telemetry.update();

         telemetry.addData("Servo Position", servoRight.getPosition());
         telemetry.update();

      }
   }
}