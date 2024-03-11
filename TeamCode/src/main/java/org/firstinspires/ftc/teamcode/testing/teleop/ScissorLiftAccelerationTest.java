package org.firstinspires.ftc.teamcode.testing.teleop;

// import stuff

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
// @Disabled
public class ScissorLiftAccelerationTest extends LinearOpMode {
   DcMotor leftScissor;
   @Override

   public void runOpMode() {

      waitForStart();
      while (opModeIsActive()) {
         leftScissor = hardwareMap.get(DcMotor.class, "left_scissor");
         if (gamepad1.a) {
            telemetry.addData("a is clicked", ":D");
            telemetry.update();
            double encoderCounts = leftScissor.getCurrentPosition();
            double power = Acceleration_Power(10, 0.1, 0.03,
                    0.1, 312, 8, 19.2,
                    537.7, encoderCounts);
            leftScissor.setPower(power);
         }
      }
   }
   /// OUR MOTOR IS 312 RPM, GEAR RATIO 19.2, MOTOR DIAMETER 8MM, ENCODER RESOLUTION 537.7
   /// NOTE TO PROGRAMMERS: NEED TO KEEP TRACK OF UNITS

   public double Acceleration_Power(double totalDistance, double targetVelocity,
                                   double initialVelocity, double accelerationProportion,
                                   double motorRPM, double motorDiameter,
                                   double gearRatio, double encoderResolution, double encoderCounts) {
      double accelerationDistance = totalDistance * accelerationProportion;
      double acceleration = ((Math.pow(targetVelocity, 2) - Math.pow(initialVelocity, 2)) / (2 * accelerationDistance));

      double motorCircumference = (motorDiameter * Math.PI) / 1000; // Given in mm, converted here to m
      double distancePerEncoderCount = motorCircumference / encoderResolution;

      double C1 = 2 * acceleration * distancePerEncoderCount;
      double C2 = Math.pow(initialVelocity, 2);

      double maxMotorVelocity = motorCircumference * gearRatio * motorRPM / 60;

      double velocity = Math.pow(C1 * encoderCounts + C2, 0.5);

      double power = velocity / maxMotorVelocity;
      telemetry.addData("power: ",power);
      telemetry.update();
      return power;

   }

}


