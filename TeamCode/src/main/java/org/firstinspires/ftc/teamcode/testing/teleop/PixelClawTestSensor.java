package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp
@Disabled
public class PixelClawTestSensor extends LinearOpMode {
    DistanceSensor sensorDistance;
    private Servo leftClaw;
    private Servo rightClaw;
    @Override
    public void runOpMode() {

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_dist");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        waitForStart();

        while (opModeIsActive()) {

            double dist = sensorDistance.getDistance(DistanceUnit.CM);
            String distString = String.format(Locale.US, "%.02f", dist);

            telemetry.addData("Distance (cm)", distString);

             if (dist < 0.7) {
                 leftClaw.setPosition(0.19);
                 rightClaw.setPosition(0.77);
                            telemetry.addData("servo left", "0.19");
                            telemetry.addData("servo right", "0.77");
                            telemetry.update();
             }

            telemetry.update();
        }
    }
}
