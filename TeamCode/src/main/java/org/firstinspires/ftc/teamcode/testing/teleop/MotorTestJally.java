package org.firstinspires.ftc.teamcode.testing.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.utility.Constants.droneHold;
import static org.firstinspires.ftc.teamcode.utility.Constants.droneLaunch;

@TeleOp

public class MotorTestJally extends LinearOpMode {
   Servo Oubloc;
   public void runOpMode() {
      Oubloc = hardwareMap.get(Servo.class, "servo");

      waitForStart();
      double pos = 0;
      while (opModeIsActive()) {

            telemetry.addData("servo position: ", Oubloc.getPosition());
            telemetry.update();
         if (gamepad1.b){
            Oubloc.setPosition(droneHold);
         }

            if (gamepad1.a){
               Oubloc.setPosition(droneLaunch);
            }
      }
   }
}

