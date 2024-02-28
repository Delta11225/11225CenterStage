package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DCEncoderTest {
    @TeleOp
    public class MotorTest extends LinearOpMode {
        private DcMotor Motor;
        @Override
        public void runOpMode() {
            Motor = hardwareMap.get(DcMotor.class, "motor");
            Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            waitForStart();
            while (opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    Motor.setPower(0.5);
                } else if (gamepad1.dpad_down){
                    Motor.setPower(-0.5);
                } else {
                    Motor.setPower(0.0);
                }
                telemetry.addData("encoder",Motor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
