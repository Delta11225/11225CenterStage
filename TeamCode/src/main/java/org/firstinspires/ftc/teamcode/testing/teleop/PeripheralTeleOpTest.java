package org.firstinspires.ftc.teamcode.testing.teleop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class PeripheralTeleOpTest extends LinearOpMode {
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
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize Motors

        //collect position
        Arm.setPosition(0.35);


        waitForStart();

        while (opModeIsActive()) {

            // Linear slide
            if (gamepad2.dpad_up&& linearSlide.getCurrentPosition() < 4100 ) {
                linearSlide.setPower(0.75);
            } else if (gamepad2.dpad_down && linearSlide.getCurrentPosition() > 0) {
                linearSlide.setPower(-0.75);
            } else {
                linearSlide.setPower(0.0);
            }

            // Collector Clamp
            if (gamepad2.right_bumper) {
                //clamp down
                Clamp.setPosition(0.75);
            }
            if (gamepad2.left_bumper) {
                //clamp up
                Clamp.setPosition(1);
            }

            // Arm
            if (gamepad2.dpad_right) {
                //collect position
                Arm.setPosition(0.35);
            } else if (gamepad2.dpad_left) {
                //deploy position
                Arm.setPosition(0.07);
            }
            telemetry.addData("encoder",linearSlide.getCurrentPosition());
            telemetry.update();

            // Arm Auto

            if(gamepad2.x) {
                linearSlide.setTargetPosition(2771);
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide.setPower(1);
                while(linearSlide.isBusy()){

                }
                Arm.setPosition(0.07);
            }
            if(gamepad2.y){
                linearSlide.setTargetPosition(4109);
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide.setPower(1);
                while(linearSlide.isBusy()){

                }
                Arm.setPosition(0.07);
            }
            if(gamepad2.a){
                Arm.setPosition(0.35);
                linearSlide.setTargetPosition(0);
                linearSlide.setPower(1);
                while(linearSlide.isBusy()){

                }
            }
        }
    }
}


