package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled // Comment or delete to use
@Autonomous(name = "IMU Turn", group = "Sensor")
public class imuTurn extends LinearOpMode
{
    // Setting up the gyro sensor
    BNO055IMU imu;
    Orientation angles;

    // Setting up motors for a mecanum drive train
    DcMotor fl, fr, bl, br;


    // Variables for power. See lines 89-90 for explanation
    int maxSpeedAngle = 120;
    float minSpeed = 0.2f;

    boolean idk = false;

    // Creating a time tracker
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        // Setting up the parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        // Setting up the gyro sensor on the hardware map and initializing the parameters
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Naming the motors, making them brake when power is set to '0', and reversing the power to some
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Waiting for start
        waitForStart();

        while (opModeIsActive()) {
            // 'targetAngle' is the final angle to end the turn at. Accepted floats are -180 to 180.
            // 'leftOrRight' is the direction to turn. accepted strings are 'left' or 'right'
            if (gamepad1.y) {
                IMUTurn(-90, "right");
            }

            // 'targetAngle' is the final angle to end the turn at. Accepted floats are -180 to 180.
            // 'leftOrRight' is the direction to turn. accepted strings are 'left' or 'right'
            if (gamepad1.a) {
                IMUTurn(45, "left");
            }

            // Telemetering our heading
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float currentHeading = angles.firstAngle;
            telemetry.addData("Heading: ", currentHeading);
            telemetry.addData("", idk);
            telemetry.update();
        }
    }

    void IMUTurn(float targetAngle, String leftOrRight) {
        // Resetting our timer
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {
            // Updating our current heading while turning
            // This records the z axis angle in the variable 'currentHeading'
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float currentHeading = angles.firstAngle;
            // Calculating the motor power. 'minSpeed' is the slowest speed to turn at.
            // 'maxSpeedAngle' is how many degrees of the angle it will turn without slowing down
            float power = Math.max(Math.abs(currentHeading-targetAngle)/maxSpeedAngle, minSpeed);

            if (Math.abs(currentHeading) < Math.abs(targetAngle)) {
                if ((Math.abs(currentHeading) - Math.abs(targetAngle)) > -.5){
                    idk = true;
                    break;
                }
            }

            if (leftOrRight.equals("left")){
                // Actually applying the power
                fl.setPower(-power);
                bl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            }
            if (leftOrRight.equals("right")){
                // Actually applying the power
                fl.setPower(power);
                bl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
            }
            // Telemetering our heading
            telemetry.addData("Heading: ", currentHeading);
            telemetry.addData("Runtime: ", runtime);
            telemetry.addData("", idk);
            telemetry.addData("Power", fl.getPower());
            telemetry.update();
        }
        // Stopping the motors once we completed our turn
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
}
