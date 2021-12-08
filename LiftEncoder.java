package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name = "Lift Encoder", group = "nsor")
public class LiftEncoder extends LinearOpMode
{
    DcMotor liftMotor, liftEncoder;

    @Override public void runOpMode() {
        // Waiting for start
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftEncoder = hardwareMap.dcMotor.get("liftEncoder");
        liftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pos: ", liftEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}