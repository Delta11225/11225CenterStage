package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RookieBotTeleOPHard extends LinearOpMode {

    private DcMotor motor0;
    private DcMotor motor1;

    @Override
    public void runOpMode() {

        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor1.setPower(-this.gamepad1.right_stick_x);
            motor0.setPower(this.gamepad1.right_stick_x);
            motor1.setPower(-this.gamepad1.right_stick_y);
            motor0.setPower(-this.gamepad1.right_stick_y);


            telemetry.addData("status", "running");
            telemetry.update();
        }
    }
}
