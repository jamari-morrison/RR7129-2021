package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Bob Auto", group="")
public class BobAuto extends LinearOpMode {
    BobBase base;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime allianceTimer = new ElapsedTime();
    int posNum = 1;

    
    @Override
    public void runOpMode() throws InterruptedException {
        base = new BobBase(this);
        while (!opModeIsActive()) {
            if (gamepad1.x) {
                posNum = 1;
            } else if (gamepad1.b) {
                posNum = 2;
            } else if (gamepad1.y) {
                posNum = 3;
            } else if (gamepad1.a) {
                posNum = 4;
            }

            if (posNum == 1) {
                telemetry.addData("Position: Blue 1 ", posNum);
                telemetry.update();
            } else if (posNum == 2) {
                telemetry.addData("Position: Red 1 ", posNum);
                telemetry.update();
            } else if (posNum == 3) {
                telemetry.addData("Position: Red 2 ", posNum);
                telemetry.update();
            } else if (posNum == 4) {
                telemetry.addData("Position: Blue 2 ", posNum);
                telemetry.update();
            }

        }

        waitForStart();

        if (opModeIsActive() && posNum ==1) {
            runtime.reset();
            base.EncoderDrive(-11, 20, .23, 3);
            base.IMUTurn(-40, "r", 120, .275f, 3);
            base.EncoderDrive(-12, 20, .23, 2);
            base.IMUTurn(-5, "l", 120, .275f, 3);
            base.EncoderDrive(14, 25, .23,2);
            base.CarouselAutoBlue();
            base.EncoderDrive(-33, 45, .23, 3);
            base.IMUTurn(87,"l", 230, .25f, 3 );
            base.EncoderDrive(-27, 40, .23,3);
            base.DeliverBlock();
            base.EncoderDrive(30, 40, .23,3);
            base.IMUTurn(3,"r", 200, .25f, 3 );
            base.EncoderDrive(10, 20, .23,3);
        } else if (opModeIsActive() && posNum == 2) {
            runtime.reset();
            base.EncoderDrive(-11, 25, .235, 3);
            base.IMUTurn(40, "l", 90, .3f, 3);
            base.EncoderDrive(-9, 25, .23, 2);
            base.IMUTurn(6, "r", 90, .3f, 3);
            base.EncoderDrive(13.5, 30, .23, 1.5f);
            base.CarouselAutoRed();
            base.EncoderDrive(-36, 45, .23, 3);
            base.IMUTurn(-80, "r", 200, .3f, 3);
            base.EncoderDrive(-32, 40, .23, 3);
            base.DeliverBlock();
            base.EncoderDrive(32, 40, .23, 3);
            base.IMUTurn(-8, "l", 150, .3f, 3);
            base.EncoderDrive(17, 20, .23, 3);
        } else if (opModeIsActive() && posNum == 3) {
            base.EncoderDrive(-22.8, 27, .23, 3);
            base.DeliverBlock();
            base.EncoderDrive(5, 9, .23, 3);
            base.IMUTurn(86, "l", 230, .275f, 3);
            base.EncoderDrive(53, 15, .99f, 5);
            base.IMUTurn(91, "l", 189, .25f, 3);
        } else if (opModeIsActive() && posNum == 4) {
            base.EncoderDrive(-22.8, 27, .23, 3);
            base.DeliverBlock();
            base.EncoderDrive(5, 9, .23, 3);
            base.IMUTurn(-83, "r", 230, .275f, 3);
            base.EncoderDrive(53, 15, .99f, 5);
            base.IMUTurn(-91, "l", 189, .25f, 3);
        }

        while (runtime.seconds() < 30 && opModeIsActive()) {
            base.Telemetry();
        }
    }
}