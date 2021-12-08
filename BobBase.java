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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class BobBase {
    OpMode opMode;


    // Setting up the gyro sensor.
    BNO055IMU imu;
    Orientation angles;


    // Declaring motors and sensors.
    DcMotor lfWheel, rfWheel, lbWheel, rbWheel, carouselSpinnerL, carouselSpinnerR, hopper, lift;
    NormalizedColorSensor colorSensorL, colorSensorR;
    View relativeLayout;

    // Creating timers.
    ElapsedTime colorSensorTimer = new ElapsedTime();
    ElapsedTime encoderDriveTimer = new ElapsedTime();
    ElapsedTime imuTimer = new ElapsedTime();
    ElapsedTime motorTimer = new ElapsedTime();
    int BarcodePosition = 0;

    // Declaring variables
    double ticksPerInch = 47.75;
    double drvTrnSpd = 1;
    double hopperSpd = 1;
    double lReading;
    double rReading;
    double veer = 1;
    double basePosNum = BobAuto.posNum;

    // Configuring motors and opMode
    public BobBase(OpMode theOpMode) {
        opMode = theOpMode;
        Configuration();
    }
    /*_________  _____________  -          ___________  |  }    ______________  |           |          |---------
    |            |            | | \     |       |       |    }  |             | |           |          |
    |            |            | |  \    |       |       |  }    |             | |           |          \--------\
    |            |            | |   \   |       |       |  \    |             | |           |                   |
    |            |            | |    \  |       |       |   \   |             | |           |                   |
    |__________  |------------- |     \ |       |       |    \  |-------------  |_________  |_________  ---------
    */
    // my brain is so dead

    // All operator controls
    public void OperatorControls() {
        // Lift controls.
        lift.setPower(opMode.gamepad2.left_stick_y);
        // Collection/delivery controls.
        hopper.setPower(-opMode.gamepad2.right_stick_y * hopperSpd);
        // Variable hopper speed.
        if (opMode.gamepad2.x) {
            hopperSpd = .75;
        } else if (opMode.gamepad2.b){
            hopperSpd = 1;
        }
    }

    // All driver controls
    public void DriverControls() {
        // Drivetrain controls
        lfWheel.setPower(opMode.gamepad1.left_stick_y - opMode.gamepad1.right_stick_x);
        rfWheel.setPower(opMode.gamepad1.left_stick_y + opMode.gamepad1.right_stick_x);
        lbWheel.setPower(opMode.gamepad1.left_stick_y - opMode.gamepad1.right_stick_x);
        rbWheel.setPower(opMode.gamepad1.left_stick_y + opMode.gamepad1.right_stick_x);
        // Increase/decrease speed. Bob go brrrrr
        if (opMode.gamepad1.dpad_up) {
            drvTrnSpd = 1;
        } else if (opMode.gamepad1.dpad_down) {
            drvTrnSpd = .75;
        }
        // Carousel spinner slow.
        if (opMode.gamepad1.left_bumper) {
            carouselSpinnerL.setPower(.5);
            carouselSpinnerR.setPower(-.5);
        }
        // Carousel spinner fast.
        else if (opMode.gamepad1.right_bumper) {
            carouselSpinnerL.setPower(.75);
            carouselSpinnerR.setPower(-.75);
        }
        // Carousel spinner off.
        else if(!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper) {
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);
        }
    }

    // try method overloading
    // Encoder drive function. DISCLAIMER: I did this quickly so it's really ugly. Don't do this next year. Like seriously, get a junk drivetrain and program on it; it'll be better than this.
    public void EncoderDrive(double inToMove, double maxSpeedDistance, double minSpeed, float timeOutB) {
        // Resetting the encoders. Encoders are plugged into the lf and rf motors. I am just too lazy to switch the ports on the control and expansion hubs.
        rbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Resetting the encoder drive timeout clock.
        encoderDriveTimer.reset();

        // Initializing the right motors app. in. reading
        double currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);

        // The important part says to keep moving until the current pos is grater than the target pos.
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentPos) < Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOutB) {
            // Calculating how much we are veering.
            veer = Math.abs(rbWheel.getCurrentPosition() / lbWheel.getCurrentPosition());
            // Constantly updating the power to the motors based on how far we have to move. This same line is used in the IMUTurn function. You can easily create proportional control by switching out a few of these variables.
            double power = Math.max(Math.abs(currentPos-inToMove)/maxSpeedDistance, minSpeed);

            // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
            if (inToMove >= 0) {
                lfWheel.setPower(-power * veer);
                lbWheel.setPower(-power * veer);
                rfWheel.setPower(-power);
                rbWheel.setPower(-power);
            } else if (inToMove < 0) {
                lfWheel.setPower(power * veer);
                lbWheel.setPower(power * veer);
                rfWheel.setPower(power);
                rbWheel.setPower(power);
            }

            // Updating current pos.
            currentPos = -(rbWheel.getCurrentPosition()/ticksPerInch);
        }

        // Stopping the drivetrain.
        lfWheel.setPower(0);
        rfWheel.setPower(0);
        lbWheel.setPower(0);
        rbWheel.setPower(0);
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
                if ((Math.abs(currentHeading) - Math.abs(targetAngle)) > -5){
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
                lbWheel.setPower(-power);
                rfWheel.setPower(power);
                rbWheel.setPower(power);
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

    // Blue autonomous path.
    public void BlueOne() {
        EncoderDrive(-16.25,50,.35,3);
        ColorSensorReadings();
        EncoderDrive(4,30,.375,3);
        IMUTurn(-84f,"r", 270, .25f, 2);
        EncoderDrive(-18.5,40,.375,3);
        IMUTurn(-9.5f,"l", 270, .25f, 2);
        EncoderDrive(4.5,30,.375,3);
        CarouselAuto();
        EncoderDrive(-36,80,.375,2);
        IMUTurn(80,"l", 270, .25f, 2);
        EncoderDrive(-29,80,.375,2);
        DeliverBlock();
        while (((LinearOpMode)opMode).opModeIsActive()) {
            Telemetry();
        }
    }

    // Reading the barcode.
    public void ColorSensorReadings() {
        // Read the barcode pos for half a second.
        colorSensorTimer.reset();
        while (colorSensorTimer.seconds() < .5 && ((LinearOpMode) opMode).opModeIsActive()) {
            lReading = ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.INCH);
            rReading = ((DistanceSensor) colorSensorR).getDistance(DistanceUnit.INCH);
        }
        if (lReading < 3) {
            BarcodePosition = 2;
        } else if (rReading < 3) {
            BarcodePosition = 1;
        } else {
            BarcodePosition = 0;
        }
        opMode.telemetry.addData("", BarcodePosition);
        opMode.telemetry.addData("L: ", lReading);
        opMode.telemetry.addData("R: ", rReading);
        opMode.telemetry.update();
    }

    public void CarouselAuto() {
        basePosNum = BobAuto.posNum;
        if (basePosNum == 1 || basePosNum == 2) {
            lfWheel.setPower(-.25);
            carouselSpinnerL.setPower(-.3);
            carouselSpinnerR.setPower(.3);
            motorTimer.reset();
            while (motorTimer.seconds() < 3 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);
            lfWheel.setPower(0);
        } else if (basePosNum == 3 || basePosNum == 4) {
            rfWheel.setPower(-.275);
            carouselSpinnerL.setPower(-.3);
            carouselSpinnerR.setPower(.3);
            motorTimer.reset();
            while (motorTimer.seconds() < 3 && ((LinearOpMode)opMode).opModeIsActive()) {
                Telemetry();
            }
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);
            rfWheel.setPower(0);
        }
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

    // Telemetry
    public void Telemetry() {
        // Telemetry
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float currentHeading = angles.firstAngle;
        opMode.telemetry.addData("Pos: ", BarcodePosition);
        opMode.telemetry.addData("Heading: ", currentHeading);
        opMode.telemetry.update();
    }

    // Device configuration
    public void Configuration() {
        // Naming motors.
        lfWheel = opMode.hardwareMap.dcMotor.get("lfWheel");
        rfWheel = opMode.hardwareMap.dcMotor.get("rfWheel");
        lbWheel = opMode.hardwareMap.dcMotor.get("lbWheel");
        rbWheel = opMode.hardwareMap.dcMotor.get("rbWheel");
        carouselSpinnerL = opMode.hardwareMap.dcMotor.get("carouselSpinnerL");
        carouselSpinnerR = opMode.hardwareMap.dcMotor.get("carouselSpinnerR");
        hopper = opMode.hardwareMap.dcMotor.get("hopper");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        // Reversing motors.
        rfWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rbWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        // Setting some motors to brake and all others to float.
        lfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rbWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        carouselSpinnerL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselSpinnerR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Setting up the color sensors for the barcode.
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

        // Setting up the IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}