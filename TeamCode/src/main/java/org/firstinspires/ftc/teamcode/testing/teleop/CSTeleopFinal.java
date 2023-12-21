package org.firstinspires.ftc.teamcode.testing.teleop;

import static org.firstinspires.ftc.teamcode.utility.Constants.armCollectPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armHoldPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armScoringPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armTrussHeight;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampClosedPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampOpenPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.droneHold;
import static org.firstinspires.ftc.teamcode.utility.Constants.droneLaunch;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployHigh;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployLow;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorHookHeightLeft;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorHookHeightRight;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorLiftHeightLeft;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorLiftHeightRight;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="CS TeleOp FINAL")
//@Disabled
public class CSTeleopFinal extends LinearOpMode {

    HardwareCC robot;



    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime lastSlideDown = new ElapsedTime();
    private final ElapsedTime lastGrab = new ElapsedTime();

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

    boolean clampIsClosed = false;
    boolean slideDown = true;
    boolean slowMode = false;

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

        //initialize drive motors
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);

        //initialize linear slide motors
        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setDirection(DcMotor.Direction.FORWARD);
        robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.linearSlide.setPower(0);
        //set slideDown == true because start game with slide down
        slideDown=true;

        //initialize scissor lift motors
        robot.leftScissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftScissor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftScissor.setPower(0);

        robot.rightScissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightScissor.setDirection(DcMotor.Direction.REVERSE);
        robot.rightScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightScissor.setPower(0);


        // Initialize Servos

        //arm set to collect position
        robot.Arm.setPosition(Constants.armCollectPosition);
        //clamp set to open position
        robot.Clamp.setPosition(clampOpenPosition);
        clampIsClosed=false;
        //drone launcher set to loaded position
        robot.Launcher.setPosition(droneHold);



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
        //TODO update robot distance sensor to 2M in config file
        //TODO test values for slow mode distance threshold
//        if (robot.RobotDistance.getDistance(DistanceUnit.CM) < 25) {
//            powerMultiplier = Constants.superSlowMultiplier;
//            telemetry.addLine("slow");
       if (ControlConfig.fast){
            powerMultiplier = Constants.fastMultiplier;
            telemetry.addLine("fast");
            slowMode=false;
        } else if (ControlConfig.slow) {
           powerMultiplier = Constants.slowMultiplier;
           telemetry.addLine("slow");
           slowMode = true;
       } else if (ControlConfig.slow && ControlConfig.fast){
            powerMultiplier = Constants.superSlowMultiplier;
            telemetry.addLine("super slow");
            slowMode = true;
        } else {
            powerMultiplier = Constants.normalMultiplier;
            telemetry.addLine("normal");
           slowMode=false;
        }
        telemetry.addData("robot distance",robot.RobotDistance.getDistance(DistanceUnit.CM));
       telemetry.addData("pixel distance",robot.Distance.getDistance(DistanceUnit.CM));
       telemetry.addData("claw Closed",clampIsClosed);
       telemetry.addData("slide down",slideDown);
        telemetry.update();


        robot.frontLeft.setPower(frontLeft * powerMultiplier);
        robot.frontRight.setPower(frontRight * powerMultiplier);
        robot.rearLeft.setPower(rearLeft * powerMultiplier);
        robot.rearRight.setPower(rearRight * powerMultiplier);


    }

    public void peripheralMove(){
        
        //code for peripheral systems

///////////////////////////////////GAMEPAD 1////////////////////////////////////



        //scissor "HOOK" automation
        //two button command to eliminate accidental trigger
        if (gamepad1.b && gamepad1.dpad_left) {
            robot.leftScissor.setTargetPosition(scissorHookHeightLeft);
            robot.rightScissor.setTargetPosition(scissorHookHeightRight);
            robot.leftScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftScissor.setPower(1);
            robot.rightScissor.setPower(1);
            while (robot.leftScissor.isBusy() || robot.rightScissor.isBusy()) {
                telemetry.addData("left encoder", robot.leftScissor.getCurrentPosition());
                telemetry.addData("right encoder", robot.rightScissor.getCurrentPosition());
                telemetry.update();
            }
            robot.leftScissor.setPower(0);
            robot.rightScissor.setPower(0);
            robot.leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //scissor "LIFT" automation
        //two button command to eliminate accidental trigger
        if (gamepad1.x && gamepad1.dpad_right) {

            robot.Arm.setPosition(armTrussHeight);

            robot.leftScissor.setTargetPosition(scissorLiftHeightLeft);
            robot.rightScissor.setTargetPosition(scissorLiftHeightRight);
            robot.leftScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftScissor.setPower(1);
            robot.rightScissor.setPower(1);
            while (robot.leftScissor.isBusy() || robot.rightScissor.isBusy()) {
                telemetry.addData("left encoder", robot.leftScissor.getCurrentPosition());
                telemetry.addData("right encoder", robot.rightScissor.getCurrentPosition());
                telemetry.update();
            }

            robot.leftScissor.setPower(0);
            robot.rightScissor.setPower(0);
            robot.leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

///////////////////////////////////GAMEPAD 2////////////////////////////////////

        //if robot has just grabbed pixels the claw is raised above the ground for quick movement to backdrop
        if(robot.linearSlide.getCurrentPosition()<200 && clampIsClosed==true && slideDown==true && lastGrab.seconds() > 1) {
            robot.Arm.setPosition(armHoldPosition);
        }

        if(robot.linearSlide.getCurrentPosition()<200 && clampIsClosed==false && slideDown==true){
            //if robot is ready to grab, but is moving quickly, the claw is raised above the ground so it doesn't drag
            if (slowMode==false){
            robot.Arm.setPosition(armHoldPosition);
            }
            //if robot is ready to grab AND moving slowly the claw is in collect position so it can collect pixels
            if (slowMode == true){
                robot.Arm.setPosition(armCollectPosition);
            }
        }

//////////////CLAMP MOVEMENTS////////////////////////////////
        // Collector Clamp Manual
        if (gamepad2.right_bumper) {
            //clamp closed
            lastGrab.reset();
            robot.Clamp.setPosition(clampClosedPosition);
            clampIsClosed=true;
        }
        if (gamepad2.left_bumper) {
            //clamp open
            lastGrab.reset();
            robot.Clamp.setPosition(clampOpenPosition);
            clampIsClosed=false;
        }

        ///// Clamp Autograb

        if (clampIsClosed==false && robot.Distance.getDistance(DistanceUnit.CM) < 1 && lastGrab.seconds()>2 && gamepad2.left_bumper==false) {
            telemetry.addData("Position", "Closed");
            telemetry.update();
            lastGrab.reset();
            robot.Clamp.setPosition(clampClosedPosition);
            gamepad2.rumble(250);//Rumble work
            clampIsClosed=true;
        }

//////////////////////Automated arm deployment///////////////////////////////////

        /////Automated deploy to LOW
        if(gamepad2.x &&  clampIsClosed==true) {
            slideDown=false;
            if (robot.linearSlide.getCurrentPosition()<1000){
                robot.Arm.setPosition(armHoldPosition);
            }
            robot.linearSlide.setTargetPosition(linearSlideAutomatedDeployLow);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linearSlide.setPower(1);
            while(robot.linearSlide.isBusy()){

            }
            robot.Arm.setPosition(armScoringPosition);
        }
        ////Automated deploy to HIGH
        if(gamepad2.y &&  clampIsClosed==true){
            slideDown=false;
            if (robot.linearSlide.getCurrentPosition()<1000){
                robot.Arm.setPosition(armHoldPosition);
            }
            robot.linearSlide.setTargetPosition(linearSlideAutomatedDeployHigh);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linearSlide.setPower(1);
            while(robot.linearSlide.isBusy()){
                //waits for arm to get to position
            }
            robot.Arm.setPosition(armScoringPosition);
        }
        ///Return to Arm collect position
        if(gamepad2.a &&  clampIsClosed==false){//add distance sensor to this later
            lastSlideDown.reset();
            robot.Arm.setPosition(armHoldPosition);
            robot.Clamp.setPosition(clampClosedPosition);
            sleep(750);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linearSlide.setTargetPosition(0);
            robot.linearSlide.setPower(1);
            while(robot.linearSlide.isBusy()&&(lastSlideDown.seconds() < 3)){
                telemetry.addData("LinearSlideEncoder",robot.linearSlide.getCurrentPosition());
                telemetry.addLine("Stuck in loop");
                telemetry.update();
            }
            telemetry.addData("LinearSlideEncoder",robot.linearSlide.getCurrentPosition());
            telemetry.update();
            robot.linearSlide.setPower(0);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Arm.setPosition(armCollectPosition);
            robot.Clamp.setPosition(clampOpenPosition);
            slideDown=true;
        }

        telemetry.addData("encoder",robot.linearSlide.getCurrentPosition());
        telemetry.update();


//////////////Launch Drone///////////////////////
        //press both triggers at the same time to launch drone
        //two button command to eliminate accidental trigger
        if (gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7) {
            //launch
            robot.Launcher.setPosition(droneLaunch);
        }




        /*
        // Linear slide manual
        if (gamepad2.dpad_up && robot.linearSlide.getCurrentPosition() < maxLinearSlidePostion ) {
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linearSlide.setPower(linearSlideUpPower);
        } else if (gamepad2.dpad_down && robot.linearSlide.getCurrentPosition() > minLinearSlidePosition) {
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linearSlide.setPower(linearSlideDownPower);
        } else {
            robot.linearSlide.setPower(0.0);
        }
  */

  /*
                // Arm MANUAL movement
        if (gamepad2.dpad_right) {
            //collect position
            robot.Arm.setPosition(armCollectPosition);
        } else if (gamepad2.dpad_left ) {
            //&& robot.linearSlide.getCurrentPosition() > 2000
            //deploy or scoring position
            robot.Arm.setPosition(armScoringPosition);
        }
*/
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