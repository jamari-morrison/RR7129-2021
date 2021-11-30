package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class BobBase {
    OpMode opMode;


    NormalizedColorSensor colorSensorL, colorSensorR;
    View relativeLayout;

    // Setting up the gyro sensor
    BNO055IMU imu;
    Orientation angles;


    // Declaring motors
    DcMotor lfWheel, rfWheel, lbWheel, rbWheel, carouselSpinnerL, carouselSpinnerR, hopper, lift;

    // Creating timer for hopper automation
    ElapsedTime hopperTimer = new ElapsedTime();



    // Declaring variables
    double hopperSpeed = 0.75;
    ElapsedTime encoderDriveTimer = new ElapsedTime();
    ElapsedTime imuTimer = new ElapsedTime();
    ElapsedTime motorTimer = new ElapsedTime();
    double ticksPerInch = 47.75;
    double drvTrnSpd = 1;
    double hopperSpd = 1;
    double lReading;
    double rReading;

    // Configuring motors and opMode
    public BobBase(OpMode theOpMode) {
        opMode = theOpMode;
        Configuration();
    }

    // All operator controls
    public void OperatorControls() {
        // Linking lift power to up/down on the left stick
        lift.setPower(opMode.gamepad2.left_stick_y);

        hopper.setPower(-opMode.gamepad2.right_stick_y * hopperSpd);

        if (opMode.gamepad2.x) {
            hopperSpd = .75;
        } else if (opMode.gamepad2.b){
            hopperSpd = 1;
        }
    }

    // Device configuration
    public void Configuration() {
        lfWheel = opMode.hardwareMap.dcMotor.get("lfWheel");
        rfWheel = opMode.hardwareMap.dcMotor.get("rfWheel");
        lbWheel = opMode.hardwareMap.dcMotor.get("lbWheel");
        rbWheel = opMode.hardwareMap.dcMotor.get("rbWheel");
        carouselSpinnerL = opMode.hardwareMap.dcMotor.get("carouselSpinnerL");
        carouselSpinnerR = opMode.hardwareMap.dcMotor.get("carouselSpinnerR");
        hopper = opMode.hardwareMap.dcMotor.get("hopper");
        lift = opMode.hardwareMap.dcMotor.get("lift");

        rfWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rbWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        lfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselSpinnerL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselSpinnerR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int relativeLayoutId = opMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", opMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) opMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        colorSensorR = opMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color_r");
        colorSensorL = opMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color_l");
        if (colorSensorL instanceof SwitchableLight) {
            ((SwitchableLight) colorSensorL).enableLight(true);
        }
        if (colorSensorL instanceof SwitchableLight) {
            ((SwitchableLight) colorSensorL).enableLight(true);
        }

        // Setting up the parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        // Setting up the gyro sensor on the hardware map and initializing the parameters
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void ColorSensorReadings() {
        lReading = ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.INCH);
        rReading = ((DistanceSensor) colorSensorR).getDistance(DistanceUnit.INCH);
        opMode.telemetry.addData("L = ", lReading);
        opMode.telemetry.addData("R = ", rReading);
        opMode.telemetry.update();
    }

    public void BlueOne() {


    }

    // All driver controls
    public void DriverControls() {
        // Drivetrain controls
        lfWheel.setPower(opMode.gamepad1.left_stick_y - opMode.gamepad1.right_stick_x);
        rfWheel.setPower(opMode.gamepad1.left_stick_y + opMode.gamepad1.right_stick_x);
        lbWheel.setPower(opMode.gamepad1.left_stick_y - opMode.gamepad1.right_stick_x);
        rbWheel.setPower(opMode.gamepad1.left_stick_y + opMode.gamepad1.right_stick_x);

        if (opMode.gamepad1.dpad_up) {
            drvTrnSpd = 1;
        } else if (opMode.gamepad1.dpad_down) {
            drvTrnSpd = .75;
        }

        if (opMode.gamepad1.left_bumper) {
            carouselSpinnerL.setPower(.5);
            carouselSpinnerR.setPower(-.5);
        }
        else if (opMode.gamepad1.right_bumper) {
            carouselSpinnerL.setPower(.75);
            carouselSpinnerR.setPower(-.75);
        }
        else if(!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper) {
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);

        }
    }

    // try method overloading
    public void EncoderDrive(double inToMove, double maxSpeedDistance, double minSpeed, float timeOutB) {
        encoderDriveTimer.reset();

        rbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentPos) < Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOutB) {
            double power = Math.max(Math.abs(currentPos-inToMove)/maxSpeedDistance, minSpeed);
            if (inToMove >= 0) {
                lfWheel.setPower(-power);
                lbWheel.setPower(-power);
                rfWheel.setPower(-power - .025);
                rbWheel.setPower(-power - .025);
            } else if (inToMove < 0) {
                lfWheel.setPower(power);
                lbWheel.setPower(power);
                rfWheel.setPower(power + .025);
                rbWheel.setPower(power + .025);
            }
            currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);
            opMode.telemetry.addData("Current pos: ", currentPos);
            opMode.telemetry.addData("inToMove", inToMove);
            opMode.telemetry.update();
        }
        opMode.telemetry.addData("Current pos: ", currentPos);
        opMode.telemetry.update();
        lfWheel.setPower(0);
        rfWheel.setPower(0);
        lbWheel.setPower(0);
        rbWheel.setPower(0);
    }

    public void CarouselAutoRed() {
        rfWheel.setPower(-.25);
        carouselSpinnerL.setPower(-.3);
        carouselSpinnerR.setPower(.3);

        motorTimer.reset();
        while (motorTimer.seconds() < 5 && ((LinearOpMode)opMode).opModeIsActive()) {
            Telemetry();
        }
        carouselSpinnerL.setPower(0);
        carouselSpinnerR.setPower(0);
        lfWheel.setPower(0);
    }

    public void CarouselAutoBlue() {
        lfWheel.setPower(-.25);
        carouselSpinnerL.setPower(-.3);
        carouselSpinnerR.setPower(.3);

        motorTimer.reset();
        while (motorTimer.seconds() < 5 && ((LinearOpMode)opMode).opModeIsActive()) {
            Telemetry();
        }
        carouselSpinnerL.setPower(0);
        carouselSpinnerR.setPower(0);
        lfWheel.setPower(0);
    }

    public void DeliverBlock() {
        motorTimer.reset();
        lift.setPower(-.6);
        while (motorTimer.seconds()<2 && ((LinearOpMode)opMode).opModeIsActive()){
            Telemetry();
        }
        lift.setPower(-.1);
        hopper.setPower(-.4);
        motorTimer.reset();
        while (motorTimer.seconds()<1 && ((LinearOpMode)opMode).opModeIsActive()) {
            Telemetry();
        }
        hopper.setPower(0);
        lift.setPower(.8);
        motorTimer.reset();
        while (motorTimer.seconds()<1 && ((LinearOpMode)opMode).opModeIsActive()){
            Telemetry();
        }
        lift.setPower(0);
    }

    void IMUTurn(float targetAngle, String leftOrRight, int maxSpeedAngle, float minSpeed, float timeOutA) {
        // Resetting our timer
        imuTimer.reset();
        while (((LinearOpMode)opMode).opModeIsActive() && imuTimer.seconds() < timeOutA) {
            // Updating our current heading while turning
            // This records the z axis angle in the variable 'currentHeading'
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float currentHeading = angles.firstAngle;
            // Calculating the motor power. 'minSpeed' is the slowest speed to turn at.
            // 'maxSpeedAngle' is how many degrees of the angle it will turn without slowing down
            float power = Math.max(Math.abs(currentHeading-targetAngle)/maxSpeedAngle, minSpeed);

            if (Math.abs(currentHeading) < Math.abs(targetAngle)) {
                if ((Math.abs(currentHeading) - Math.abs(targetAngle)) > -.7){
                    break;
                }
            }

            if (leftOrRight.equals("left") || leftOrRight.equals("l")){
                // Actually applying the power
                lfWheel.setPower(power);
                lbWheel.setPower(power);
                rfWheel.setPower(-power);
                rbWheel.setPower(-power);
            } else {
                // Actually applying the power
                lfWheel.setPower(-power);
                rfWheel.setPower(power);
            }
            opMode.telemetry.addData("Heading: ", currentHeading);
            opMode.telemetry.update();
        }
        // Stopping the motors once we completed our turn
        lfWheel.setPower(0);
        lbWheel.setPower(0);
        rfWheel.setPower(0);
        rbWheel.setPower(0);
    }

    // Telemetry
    public void Telemetry() {
        // Telemetry
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float currentHeading = angles.firstAngle;
        double currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);

        opMode.telemetry.addData("Heading: ", currentHeading);
        opMode.telemetry.update();
    }
}