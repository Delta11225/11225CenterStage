package org.firstinspires.ftc.teamcode.testing.teleop;
import static org.firstinspires.ftc.teamcode.utility.Constants.armScoringPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armTrussHeight;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployLow;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorHookHeightLeft;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorHookHeightRight;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorLiftHeightLeft;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorLiftHeightRight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.HardwareCC;

@TeleOp
@Disabled
public class ScissorTest extends LinearOpMode {
    HardwareCC robot;
    private DcMotor leftScissor;
    private DcMotor rightScissor;

    @Override
    public void runOpMode() {
        robot = new HardwareCC(hardwareMap);
        leftScissor = hardwareMap.get(DcMotor.class, "left_scissor");
        leftScissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftScissor.setDirection(DcMotor.Direction.REVERSE);
        leftScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightScissor = hardwareMap.get(DcMotor.class, "right_scissor");
        rightScissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightScissor.setDirection(DcMotor.Direction.REVERSE);
        rightScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b && gamepad1.dpad_left) {
                leftScissor.setTargetPosition(scissorHookHeightLeft);
                rightScissor.setTargetPosition(scissorHookHeightRight);
                leftScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftScissor.setPower(1);
                rightScissor.setPower(1);
                while (leftScissor.isBusy() || rightScissor.isBusy()) {
                    telemetry.addData("left encoder", leftScissor.getCurrentPosition());
                    telemetry.addData("right encoder", rightScissor.getCurrentPosition());
                    telemetry.update();
                }
                leftScissor.setPower(0);
                rightScissor.setPower(0);
                leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
                if (gamepad1.x && gamepad1.dpad_right) {

                    robot.Arm.setPosition(armTrussHeight);

                    leftScissor.setTargetPosition(scissorLiftHeightLeft);
                    rightScissor.setTargetPosition(scissorLiftHeightRight);
                    leftScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftScissor.setPower(1);
                    rightScissor.setPower(1);
                    while (leftScissor.isBusy() || rightScissor.isBusy()) {
                        telemetry.addData("left encoder", leftScissor.getCurrentPosition());
                        telemetry.addData("right encoder", rightScissor.getCurrentPosition());
                        telemetry.update();
                    }

                    leftScissor.setPower(0);
                    rightScissor.setPower(0);
                    leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                 /*
                } if (gamepad1.dpad_up) {
                    leftScissor.setPower(1);

                } if (gamepad1.y) {
                    rightScissor.setPower(1);

                } if (gamepad1.dpad_down) {
                    leftScissor.setPower(-0.75);

                } if (gamepad1.a) {
                    rightScissor.setPower(-0.75);

                } else {
                    leftScissor.setPower(0.0);
                    rightScissor.setPower(0.0);

                  */

                telemetry.addData("left encoder", leftScissor.getCurrentPosition());
                telemetry.addData("right encoder", rightScissor.getCurrentPosition());
                telemetry.update();
            }
        }
    }


