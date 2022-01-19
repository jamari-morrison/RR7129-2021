package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous(name = "Lift Encoder", group = "nsor")
public class LiftEncoder extends LinearOpMode
{
    DcMotor lift;

    @Override public void runOpMode() {
        // Waiting for start
        lift = hardwareMap.dcMotor.get("lift");
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pos: ", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}