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
//Init that the position is at 0.8 (hold position)
        Launcher.setPosition(0.8);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b) {
                //hold
                Launcher.setPosition(0.8);
            }
            if (gamepad1.a) {
                //launch
                Launcher.setPosition(1);
            }
                telemetry.addData("status", "running");
                telemetry.update();
            }
        }
    }
