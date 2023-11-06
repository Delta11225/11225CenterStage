package org.firstinspires.ftc.teamcode.testing.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CSTeleop extends LinearOpMode {
    // Claw open and close
    private Servo Clamp;
    private Servo Arm;
    private DcMotor linearSlide;
    private Servo Collector;


    @Override
    public void runOpMode() throws InterruptedException {
        // Claws open and close
        Clamp = hardwareMap.get(Servo.class, "clamp");
        Arm = hardwareMap.get(Servo.class, "arm");

        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.b) {
                //clamp down
                Clamp.setPosition(0.75);

            }
            if (gamepad2.a) {
                //clamp up
                Clamp.setPosition(1);

            }

            if (gamepad2.dpad_right) {
                //deploy position
                Arm.setPosition(0.35);
            } else if (gamepad2.dpad_left) {
                //collect position
                Arm.setPosition(0.07);
            }
        }
        // Potentially correct Arm and Linear Slide code


    }
}


