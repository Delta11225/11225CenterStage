package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
@Disabled
public class ArmTest extends LinearOpMode {
   private DcMotor Arm;
   @Override
   public void runOpMode() {
     Arm = hardwareMap.get(DcMotor.class, "arm");
     Arm .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.y) {
            Arm.setPower(0.5);
         } else if (gamepad1.a){
            Arm.setPower(-0.5);
         } else {
            Arm.setPower(0.0);
         }
         telemetry.addData("encoder",Arm.getCurrentPosition());
         telemetry.update();
      }
   }
}
