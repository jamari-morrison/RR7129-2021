package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="Bob Auto", group="")
public class BobAuto extends LinearOpMode {
    BobBase base;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime allianceTimer = new ElapsedTime();
    static int posNum = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        base = new BobBase(this);
        while (!opModeIsActive()) {
            if (gamepad1.x) {
                posNum = 1;
            } else if (gamepad1.a) {
                posNum = 2;
            } else if (gamepad1.b) {
                posNum = 3;
            } else if (gamepad1.y) {
                posNum = 4;
            }

            if (posNum == 1) {
                telemetry.addData("Blue 1 ", "");
                telemetry.update();
            } else if (posNum == 2) {
                telemetry.addData("Blue 2 ", "");
                telemetry.update();
            } else if (posNum == 3) {
                telemetry.addData("Red 1 ", "");
                telemetry.update();
            } else if (posNum == 4) {
                telemetry.addData("Red 2 ", "");
                telemetry.update();
            }

        }
        waitForStart();
        if (posNum == 1) {
            base.BlueOne();
        }
        else if (posNum == 2) {
            base.BlueTwo();
        }
        else if (posNum == 3) {
            base.RedOne();
        }
        else if (posNum == 4) {
            base.RedTwo();
        }
        while (opModeIsActive()) {
            base.Telemetry();
        }
    }
}