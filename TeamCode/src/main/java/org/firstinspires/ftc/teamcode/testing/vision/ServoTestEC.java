package org.firstinspires.ftc.teamcode.testing.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTestEC extends LinearOpMode {


    private Servo Servo;

    @Override
    public void runOpMode() {

        Servo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x){
                Servo.setPosition(0);
                telemetry.addData("servo left", "0");
                telemetry.addData("servo right", "0.98");
                telemetry.update();
            }
            if (gamepad1.b){
                Servo.setPosition(0);
                telemetry.addData("servo left", "0.19");
                telemetry.addData("servo right", "0.77");
                telemetry.update();
            }
        }
    }
}