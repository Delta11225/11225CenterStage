package org.firstinspires.ftc.teamcode.testing.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
@Disabled
public class ServoTestEC extends LinearOpMode {
    private Servo servoTest;

    @Override

    public void runOpMode() {
        servoTest = hardwareMap.get(Servo.class, "servo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
// move to 0 degrees.
                servoTest.setPosition(0);
            }
            if (gamepad1.y) {
// move to 0 degrees.
                servoTest.setPosition(0.5);
            }
            if (gamepad1.b) {
// move to 180 degrees.
                servoTest.setPosition(1);
            }
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.update();
        }
    }
}