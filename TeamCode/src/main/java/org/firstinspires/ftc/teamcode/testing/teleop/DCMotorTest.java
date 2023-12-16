package org.firstinspires.ftc.teamcode.testing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.HardwareCC;

@TeleOp
@Disabled
public class DCMotorTest extends LinearOpMode {
    //HardwareCC robot;
    private DcMotor motor0;
    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;


    @Override
    public void runOpMode() {
        //motor0=hardwareMap.dcMotor.get("motor0");
        rearLeft = hardwareMap.dcMotor.get("rear_left");
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        rearRight = hardwareMap.dcMotor.get("rear_right");

        //motor0.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //motor0.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);




        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x){
                //motor0.setPower(1);
               frontLeft.setPower(1);
                frontRight.setPower(1);
                rearLeft.setPower(1);
                rearRight.setPower(1);

            }
            else if(gamepad1.b){
                //motor0.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                rearLeft.setPower(-1);
                rearRight.setPower(-1);
            }
            else{
                //motor0.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                rearLeft.setPower(0);
                rearRight.setPower(0);
            }


            //telemetry.addData("power", motor0.getPower());
            telemetry.update();
        }
    }
}
