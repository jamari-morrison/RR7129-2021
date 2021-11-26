package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Encoder", group = "Sensor")
public class TankEncoder extends LinearOpMode
{
    double ticksPerInch = 47.75;
    double wheelCircumference = 12.56;
    double gearRatio = 1.67;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime encoderDriveTimer = new ElapsedTime();
    DcMotor lfWheel, rfWheel, lbWheel, rbWheel;


    @Override public void runOpMode() {


        lfWheel = hardwareMap.dcMotor.get("lfWheel");
        rfWheel = hardwareMap.dcMotor.get("rfWheel");
        lbWheel = hardwareMap.dcMotor.get("lbWheel");
        rbWheel = hardwareMap.dcMotor.get("rbWheel");

        rfWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rbWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        lfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Waiting for start
        waitForStart();

        while (opModeIsActive()) {

            double currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);
            telemetry.addData("Right: ", -rbWheel.getCurrentPosition());
            telemetry.addData("Runtime: ", runtime);
            telemetry.addData("Current pos: ", currentPos);
            telemetry.update();

            if (gamepad1.a) {
                rbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad1.y) {
                EncoderDrive(6, 0, 0);
            }


        }
    }

    public void EncoderDrive(double inToMove, double maxSpeedDistance, double minSpeed) {
        encoderDriveTimer.reset();

        double currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);
        while (opModeIsActive() && currentPos < inToMove && encoderDriveTimer.seconds() < 5) {
            lfWheel.setPower(-.3);
            rfWheel.setPower(-.4);
            currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);
            telemetry.addData("Current pos: ", currentPos);
            telemetry.addData("inToMove", inToMove);
            telemetry.update();
        }
        lfWheel.setPower(0);
        rfWheel.setPower(0);
        lbWheel.setPower(0);
        rbWheel.setPower(0);
        rbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}