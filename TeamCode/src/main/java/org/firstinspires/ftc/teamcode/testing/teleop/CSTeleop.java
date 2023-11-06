package org.firstinspires.ftc.teamcode.testing.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CSTeleop extends LinearOpMode{
    // Claw open and close
    private Servo Clamp;
    private Servo Arm;
    private DcMotor linearSlide;
    private Servo Collector;


    @Override
    public void runOpMode() throws InterruptedException {
        // Claws open and close
        Clamp = hardwareMap.get(Servo.class, "clamp");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if (gamepad2.b){
                //clamp down
                Clamp.setPosition(0);

            }
            if(gamepad2.a){
                //clamp up
               Clamp.setPosition(1);

            }
        }
        // Potentially correct Arm and Linear Slide code
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm = hardwareMap.get(Servo.class,"arm" );

        // code needed for camera to display on FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        //FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();

        Arm.setPosition(0);



        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_up&& linearSlide.getCurrentPosition() < 1900 ) {
                linearSlide.setPower(0.5);
            } else if (gamepad2.dpad_down && linearSlide.getCurrentPosition() > 0) {
                linearSlide.setPower(-0.5);
            } else {
                linearSlide.setPower(0.0);
            }

            }

            if (gamepad2.dpad_right) {
                //deploy position
                Arm.setPosition(0.5);
            }
            else if (gamepad2.dpad_left) {
                //collect position
                Arm.setPosition(0);
            }

            if(gamepad2.a &&linearSlide.getCurrentPosition() < 120){



            }

            telemetry.addData("encoder",linearSlide.getCurrentPosition());
            telemetry.update();
        }


    }


