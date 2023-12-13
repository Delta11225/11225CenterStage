package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmTestServo extends LinearOpMode {

    private Servo arm;


    @Override
    public void runOpMode() {

        arm = hardwareMap.get(Servo.class, "arm");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x){
                //deploy
               arm.setPosition(0.6);


            }
            if (gamepad1.b){
                //collect
                arm.setPosition(0.93);

            }
        }
    }
}
