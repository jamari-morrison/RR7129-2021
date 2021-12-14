package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

class RedMiniBotBase {
    OpMode opMode;
    DcMotor leftWheel, rightWheel;

    public RedMiniBotBase(OpMode theOpMode) {
        opMode = theOpMode;
        rightWheel = opMode.hardwareMap.dcMotor.get("leftWheel");
        leftWheel = opMode.hardwareMap.dcMotor.get("rightWheel");

        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void movingDaPushBot () {
        leftWheel.setPower(-opMode.gamepad1.left_stick_y); //Make - if red
        rightWheel.setPower(-opMode.gamepad1.left_stick_y); //Make - if red

        leftWheel.setPower(-opMode.gamepad1.right_stick_x);
        rightWheel.setPower(opMode.gamepad1.right_stick_x);
    }

    public void stop() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
}

