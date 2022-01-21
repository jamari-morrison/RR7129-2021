package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous(name = "Lift Encoder", group = "nsor")
public class LiftEncoder extends LinearOpMode
{
    DcMotor liftM, lfDrvtrnM, rfDrvtrnM, lbDrvtrnM, rbDrvtrnM;

    @Override public void runOpMode() {
        // Waiting for start
        liftM = hardwareMap.dcMotor.get("liftM");
        liftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrvtrnM = hardwareMap.dcMotor.get("lfDrvtrnM");
        lfDrvtrnM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrvtrnM = hardwareMap.dcMotor.get("rfDrvtrnM");
        rfDrvtrnM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrvtrnM = hardwareMap.dcMotor.get("lbDrvtrnM");
        lbDrvtrnM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrvtrnM = hardwareMap.dcMotor.get("rbDrvtrnM");
        rbDrvtrnM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Lift Pos: ", liftM.getCurrentPosition());
            telemetry.addData("lfDrvtrnM Pos: ", lfDrvtrnM.getCurrentPosition());
            telemetry.addData("rfDrvtrnM Pos: ", rfDrvtrnM.getCurrentPosition());
            telemetry.addData("lbDrvtrnM Pos: ", lbDrvtrnM.getCurrentPosition());
            telemetry.addData("rbDrvtrnM Pos: ", rbDrvtrnM.getCurrentPosition());
            telemetry.update();
        }
    }
}