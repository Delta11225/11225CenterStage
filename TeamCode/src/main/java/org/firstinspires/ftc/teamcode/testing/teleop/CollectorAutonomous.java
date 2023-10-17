package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// NEEDS TO BE MOVED INTO CORRECT AUTONOMOUS FOLDER
// UNTESTED CODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

@Autonomous
public class CollectorAutonomous extends LinearOpMode {
   private Servo Collector;

   @Override
   public void runOpMode() {

      Collector = hardwareMap.get(Servo.class, "tax_collector");
      telemetry.addData("Status", "Initialized");
      telemetry.update();

      waitForStart();

      while (opModeIsActive()) {
         // FIGURE OUT POSITION ONCE ATTACHED! (needs to rotate about 210 degrees counterclockwise)
         Collector.setPosition(0);
         telemetry.addData("status", "running");
         telemetry.update();
      }
   }
}


