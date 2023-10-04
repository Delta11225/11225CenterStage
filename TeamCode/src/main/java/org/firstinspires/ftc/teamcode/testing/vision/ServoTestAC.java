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

      servoLeft.setPosition(0.3);
      telemetry.addData("Motor Power", servoLeft.getPosition());

      telemetry.update();
      sleep(50);
      servoRight.setPosition(0.3);
      telemetry.addData("Motor Power", servoRight.getPosition());
      telemetry.update();

   }
}