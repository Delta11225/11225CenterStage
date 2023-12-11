package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CollectionServoTest extends LinearOpMode {
   private CRServo Collector;
   @Override
   public void runOpMode() {
     Collector = hardwareMap.get(CRServo.class, "collector");

      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.y) {
            Collector.setPower(1);

         } else if (gamepad1.a){
             Collector.setPower(-1);
         } else {
             Collector.setPower(0);
         }
         telemetry.addData("collector power",Collector.getPower());
         telemetry.update();
      }
   }
}
