package org.firstinspires.ftc.teamcode.testing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class DroneTest extends LinearOpMode {

    private Servo Launcher;


    @Override
    public void runOpMode() {

        Launcher = hardwareMap.get(Servo.class, "launcher");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //code needed for camera to display on FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        //FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();
        waitForStart();

        if (gamepad1.a) {
            Launcher.setPosition(.5);
        }
        else {
            Launcher.setPosition(0);
        }
        telemetry.addData("status", "running");
        telemetry.update();
    }
}

