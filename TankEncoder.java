package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder", group = "Sensor")
public class TankEncoder extends LinearOpMode
{
    DcMotor lfWheel, rfWheel, lbWheel, rbWheel;


    @Override public void runOpMode() {


        lfWheel = hardwareMap.dcMotor.get("lfWheel");
        rfWheel = hardwareMap.dcMotor.get("rfWheel");
        lbWheel = hardwareMap.dcMotor.get("lbWheel");
        rbWheel = hardwareMap.dcMotor.get("rbWheel");

        rbWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rfWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        lfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Waiting for start
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                lfWheel.setPower(.5);
                lbWheel.setPower(.5);
                rbWheel.setPower(.5);
                rfWheel.setPower(.5);
            } else {
                lfWheel.setPower(.0);
                lbWheel.setPower(.0);
                rbWheel.setPower(.0);
                rfWheel.setPower(.0);
            }
        }
    }
}