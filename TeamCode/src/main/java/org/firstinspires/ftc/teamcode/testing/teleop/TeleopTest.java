package org.firstinspires.ftc.teamcode.testing.teleop;

import static org.firstinspires.ftc.teamcode.utility.Constants.armCollectPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armScoringPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampDownPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampUpPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployHigh;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployLow;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideDownPower;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideUpPower;
import static org.firstinspires.ftc.teamcode.utility.Constants.maxLinearSlidePostion;
import static org.firstinspires.ftc.teamcode.utility.Constants.minLinearSlidePosition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.utility.Constants;
import org.firstinspires.ftc.teamcode.utility.HardwareCC;
import org.firstinspires.ftc.teamcode.utility.ControlConfig;

import java.util.Locale;

@TeleOp(name="TeleOp test")
//@Disabled
public class TeleopTest extends LinearOpMode {

    HardwareCC robot;



    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    double frontLeft;
    double rearLeft;
    double frontRight;
    double rearRight;

    double forward;
    double right;
    double clockwise;

    double powerMultiplier = 1;
    double deadZone = Math.abs(0.2);

    double temp;
    double side;

    double currentAngle;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);






        composeTelemetry();

        robot = new HardwareCC(hardwareMap);

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);
        //robot.dumpServo.setPosition(0);

        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setDirection(DcMotor.Direction.FORWARD);
        robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Motors

        //collect position
        robot.Arm.setPosition(Constants.armCollectPosition);

        robot.Clamp.setPosition(clampUpPosition);

        // End init phase
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            ControlConfig.update(gamepad1, gamepad2);

            telemetry.update();

            while (angles.firstAngle < 0 && opModeIsActive()) {
                ControlConfig.update(gamepad1, gamepad2);

                telemetry.update();
                move();
                peripheralMove();

                currentAngle = angles.firstAngle + 360;
                telemetry.addData("currentAngle loop 1", "%.1f", currentAngle);
            }

            while (angles.firstAngle >= 0 && opModeIsActive()) {
                ControlConfig.update(gamepad1, gamepad2);

                telemetry.update();
                move();
                peripheralMove();

                currentAngle = angles.firstAngle;
                telemetry.addData("currentAngle loop 2", "%.1f", currentAngle);
            }

            telemetry.addLine("null angle");
        }
    }


    public void move(){
        ControlConfig.update(gamepad1, gamepad2);
        double theta = Math.toRadians(currentAngle);

        telemetry.addData("CurrentAngle", currentAngle);
        telemetry.addData("Theta", theta);

        forward = ControlConfig.forward;
        right = ControlConfig.right;
        clockwise = ControlConfig.clockwise;

        temp = (forward * Math.cos(theta) - right * Math.sin(theta));
        side = (forward * Math.sin(theta) + right * Math.cos(theta));

        forward = temp;
        right = side;

        telemetry.addData("right: ", right);
        telemetry.addData("forward: ", forward);
        telemetry.addData("temp: ", temp);
        telemetry.addData("side: ", side);
        telemetry.addData("clockwise: ", clockwise);

        frontLeft = forward + right + clockwise;
        rearLeft = forward - right + clockwise;
        rearRight = forward + right - clockwise;
        frontRight = forward - right - clockwise;

        telemetry.addData("front left: ", frontLeft);
        telemetry.addData("rear left: ", rearLeft);
        telemetry.addData("rear right: ", rearRight);
        telemetry.addData("front right: ", frontRight);

        // Handle speed control
        if (ControlConfig.fast){
            powerMultiplier = Constants.fastMultiplier;
            telemetry.addLine("fast");
        } else if (ControlConfig.slow) {
            powerMultiplier = Constants.slowMultiplier;
            telemetry.addLine("slow");
        } else {
            powerMultiplier = Constants.normalMultiplier;
            telemetry.addLine("normal");
        }

        telemetry.update();


        robot.frontLeft.setPower(frontLeft * powerMultiplier);
        robot.frontRight.setPower(frontRight * powerMultiplier);
        robot.rearLeft.setPower(rearLeft * powerMultiplier);
        robot.rearRight.setPower(rearRight * powerMultiplier);


    }

    public void peripheralMove(){
        
        //code for peripheral systems
        // Linear slide
        if (gamepad2.dpad_up&& robot.linearSlide.getCurrentPosition() < maxLinearSlidePostion ) {
            robot.linearSlide.setPower(linearSlideUpPower);
        } else if (gamepad2.dpad_down && robot.linearSlide.getCurrentPosition() > minLinearSlidePosition) {
            robot.linearSlide.setPower(linearSlideDownPower);
        } else {
            robot.linearSlide.setPower(0.0);
        }

        // Collector Clamp
        if (gamepad2.right_bumper) {
            //clamp down
            robot.Clamp.setPosition(clampDownPosition);
        }
        if (gamepad2.left_bumper) {
            //clamp up
            robot.Clamp.setPosition(clampUpPosition);
            sleep(1000);
        }

        //////////////////////Automated arm deployment///////////////////////////////////

        /////Automated deploy to LOW
        if(gamepad2.x) {
            robot.linearSlide.setTargetPosition(linearSlideAutomatedDeployLow);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linearSlide.setPower(1);
            while(robot.linearSlide.isBusy()){

            }
            robot.Arm.setPosition(armScoringPosition);
        }
        ////Automated deploy to HIGH
        if(gamepad2.y){
            robot.linearSlide.setTargetPosition(linearSlideAutomatedDeployHigh);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linearSlide.setPower(1);
            while(robot.linearSlide.isBusy()){

            }
            robot.Arm.setPosition(armScoringPosition);
        }
        ///Return to Arm collect position
        if(gamepad2.a){
            robot.Arm.setPosition(armCollectPosition);
            robot.linearSlide.setTargetPosition(0);
            robot.linearSlide.setPower(1);
            while(robot.linearSlide.isBusy()){

            }
        }


        // Arm MANUAL movement
        if (gamepad2.dpad_right) {
            //collect position
            robot.Arm.setPosition(armCollectPosition);
        } else if (gamepad2.dpad_left ) {
            //&& robot.linearSlide.getCurrentPosition() > 2000
            //deploy or scoring position
            robot.Arm.setPosition(armScoringPosition);
        }
        telemetry.addData("encoder",robot.linearSlide.getCurrentPosition());
        telemetry.update();


        ///// Clamp Autograb

        if (robot.Clamp.getPosition() == 1 && robot.Distance.getDistance(DistanceUnit.CM) < 1) {
            telemetry.addData("Position", "Closed");
            telemetry.update();
            robot.Clamp.setPosition(0.75);

            gamepad2.rumble(250);//Rumble work
            sleep(1000);
        }
    }





    /*-----------------------------------//
    * DO NOT WRITE CODE BELOW THIS LINE  *
    * -----------------------------------*/
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

        // telemetry.addData("currentAngle", "%.1f", currentAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}