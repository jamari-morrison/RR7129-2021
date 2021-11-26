package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp (name = "Hopper Stopper",group = "2021")
public class HopperStopper extends LinearOpMode {
    ElapsedTime hopperTimer = new ElapsedTime();
    boolean isOn = false;


    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Is on = ", isOn);
            telemetry.update();
            if (gamepad1.a) {
                telemetry.addData("Is on = ", isOn);
                telemetry.update();
                isOn = true;
            }
            HopperStopper();
        }

    }
    public void HopperStopper(){
        telemetry.addData("Is on = ", isOn);
        telemetry.update();
        if (hopperTimer.seconds()>10 && gamepad1.y) {
            hopperTimer.reset();
            isOn = false;
            telemetry.addData("Is on = ", isOn);
            telemetry.update();
        }
    }
}