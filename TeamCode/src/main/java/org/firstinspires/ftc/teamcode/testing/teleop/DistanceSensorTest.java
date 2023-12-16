package org.firstinspires.ftc.teamcode.testing.teleop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
@Disabled
public class DistanceSensorTest extends LinearOpMode {
    // Claw open and close

    private DistanceSensor Distance;

    @Override
    public void runOpMode() throws InterruptedException {

        Distance = hardwareMap.get(DistanceSensor.class, "robot_distance");

        waitForStart();
        while (opModeIsActive()) {

                telemetry.addData("Distance", Distance.getDistance(DistanceUnit.CM));
                telemetry.update();


        }
    }
}