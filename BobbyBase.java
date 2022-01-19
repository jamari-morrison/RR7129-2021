package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
class BobbyBase {
    OpMode opMode;

    DcMotor collectionM, liftM, lSpinnerM, rSpinnerM, lfDrvtrnM, rfDrvtrnM, lbDrvtrnM, rbDrvtrnM;
    Servo deliveryS;

    ElapsedTime imuTimer = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;
    float currentHeading = 0;

    public BobbyBase(OpMode theOpMode){
        opMode = theOpMode;


        collectionM = opMode.hardwareMap.dcMotor.get("collectionM");
        liftM = opMode.hardwareMap.dcMotor.get("liftM");
        lSpinnerM = opMode.hardwareMap.dcMotor.get("lSpinnerM");
        rSpinnerM = opMode.hardwareMap.dcMotor.get("rSpinnerM");
        lfDrvtrnM = opMode.hardwareMap.dcMotor.get("lfDrvtrnM");
        rfDrvtrnM = opMode.hardwareMap.dcMotor.get("rfDrvtrnM");
        lbDrvtrnM = opMode.hardwareMap.dcMotor.get("lbDrvtrnM");
        rbDrvtrnM = opMode.hardwareMap.dcMotor.get("rbDrvtrnM");
        this.deliveryS = opMode.hardwareMap.get(Servo.class, "deliveryS");

        collectionM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrvtrnM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrvtrnM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrvtrnM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrvtrnM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rfDrvtrnM.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrvtrnM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void DriverControls() {
        if (opMode.gamepad1.left_bumper) {
            lSpinnerM.setPower(-.5);
            rSpinnerM.setPower(.5);
        }
        // Carousel spinner fast.
        else if (opMode.gamepad1.right_bumper) {
            lSpinnerM.setPower(-.35);
            rSpinnerM.setPower(.35);
        }
        // Carousel spinner off.
        else if(!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper) {
            lSpinnerM.setPower(0);
            rSpinnerM.setPower(0);
        }


        double y = -opMode.gamepad1.left_stick_y; // Remember, this is reversed!
        double x = opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = opMode.gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        lfDrvtrnM.setPower(frontLeftPower);
        lbDrvtrnM.setPower(backLeftPower);
        rfDrvtrnM.setPower(frontRightPower);
        rbDrvtrnM.setPower(backRightPower);
    }

    // All operator controls
    public void OperatorControls() {
        // Lift controls.
        liftM.setPower(opMode.gamepad2.left_stick_y);
        // Collection/delivery controls.
        collectionM.setPower(opMode.gamepad2.right_stick_y);

        if (opMode.gamepad2.y) {
            deliveryS.setPosition(100);
            opMode.telemetry.addData("on","");
            opMode.telemetry.update();
        } else if (opMode.gamepad2.a) {
            deliveryS.setPosition(0);
            opMode.telemetry.addData("off","");
            opMode.telemetry.update();
        }
    }















    void IMUTurn(float targetAngle, String leftOrRight, int maxSpeedAngle, float minSpeed, float timeOutA) {
        // Resetting our timer
        imuTimer.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& imuTimer.seconds() < timeOutA) {
            // Updating our current heading while turning
            // This records the z axis angle in the variable 'currentHeading'
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (angles.thirdAngle);
            // Calculating the motor power. 'minSpeed' is the slowest speed to turn at.
            // 'maxSpeedAngle' is how many degrees of the angle it will turn without slowing down
            float power = Math.max(Math.abs(currentHeading-targetAngle)/maxSpeedAngle, minSpeed);

            if (Math.abs(currentHeading) < Math.abs(targetAngle)) {
                if ((Math.abs(currentHeading) - Math.abs(targetAngle)) > -2.5){
                    break;
                }
            }

            if (leftOrRight.equals("left") || leftOrRight.equals("l")){
                // Actually applying the power
                lfDrvtrnM.setPower(-power);
                lbDrvtrnM.setPower(-power);
                rfDrvtrnM.setPower(power);
                rbDrvtrnM.setPower(power);
            } else {
                // Actually applying the power
                lfDrvtrnM.setPower(power);
                lbDrvtrnM.setPower(power);
                rfDrvtrnM.setPower(-power);
                rbDrvtrnM.setPower(-power);
            }
            opMode.telemetry.addData("Heading: ", currentHeading);
            opMode.telemetry.update();
        }
        // Stopping the motors once we completed our turn
        lfDrvtrnM.setPower(0);
        lbDrvtrnM.setPower(0);
        rfDrvtrnM.setPower(0);
        rbDrvtrnM.setPower(0);
    }

    public void resetEncoders(){
        lfDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrvtrnM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Returns true if we are in initialization
    boolean isInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }

    public void sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }
}