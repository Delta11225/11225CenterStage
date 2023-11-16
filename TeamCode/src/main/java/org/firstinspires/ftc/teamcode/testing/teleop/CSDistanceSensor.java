package org.firstinspires.ftc.teamcode.testing.teleop;
import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class CSDistanceSensor extends LinearOpMode {
    // Claw open and close
    private Servo Clamp;
    private DistanceSensor Distance;

    @Override
    public void runOpMode() throws InterruptedException {
        // Claws open and close
        Clamp = hardwareMap.get(Servo.class, "clamp");
        Distance = hardwareMap.get(DistanceSensor.class, "distance");

        waitForStart();
        while (opModeIsActive()) {


            if (Clamp.getPosition() == 1 && Distance.getDistance(DistanceUnit.CM) > 3) {

            }

        }
    }
}