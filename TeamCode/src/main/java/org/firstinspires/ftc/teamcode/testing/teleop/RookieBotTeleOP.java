package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RookieBotTeleOP extends LinearOpMode {

    private DcMotor motor0;
    private DcMotor motor1;

    @Override
    public void runOpMode() {

        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor0.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor0.setPower(-this.gamepad1.left_stick_y);
            motor1.setPower(-this.gamepad1.right_stick_y);
            telemetry.addData("status", "running");
            telemetry.update();
        }
    }
}
