package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class PixelClawTest extends LinearOpMode {

    private Servo leftClaw;
    private Servo rightClaw;

    @Override
    public void runOpMode() {

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x){
                leftClaw.setPosition(0);
                rightClaw.setPosition(0.98);
                telemetry.addData("servo left", "0");
                telemetry.addData("servo right", "0.98");
                telemetry.update();
            }
            if (gamepad1.b){
                leftClaw.setPosition(0.19);
                rightClaw.setPosition(0.77);
                telemetry.addData("servo left", "0.19");
                telemetry.addData("servo right", "0.77");
                telemetry.update();
            }
        }
    }
}
