package org.firstinspires.ftc.teamcode.testing.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MotorTestJally extends LinearOpMode {
   Servo Oubloc;
   public void runOpMode() {
      waitForStart();
      double pos = 0;
      while (opModeIsActive()) {
         Oubloc = hardwareMap.get(Servo.class, "servo");
            telemetry.addData("servo position: ", Oubloc.getPosition());
            telemetry.update();
         if (gamepad1.b){
            Oubloc.setPosition(0.0);
         }
         if (gamepad1.x){
            Oubloc.setPosition(0.21);
         }
            if (gamepad1.a){
               Oubloc.setPosition(0.52);
            }
      }
   }
}

