package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PixelClaw2Test extends LinearOpMode {


    private Servo rightClaw;

    @Override
    public void runOpMode() {


        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x){
            //closed
                rightClaw.setPosition(0.5);

                telemetry.addData("servo right", "1");
                telemetry.update();
            }
            if (gamepad1.b){
                //open
                rightClaw.setPosition(0);

                telemetry.addData("servo right", "0");
                telemetry.update();
            }
        }
    }
}
