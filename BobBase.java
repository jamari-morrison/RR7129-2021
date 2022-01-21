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

    // Declaring variables
    boolean upFlag = false;
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    double ticksPerInch = 47.75;
    double drvTrnSpd = .75;
    double hopperSpd = 1;
    double lReading = 0;
    double rReading = 0;
    double veer = 1;
    double basePosNum = BobAuto.posNum;
    int BarcodePosition = 0;
    float currentHeading = 0;
    float headingAdjustment = 0;
    float adjCurrentHeading = 0;
    double currentPosInches = 0;
    double left = 0;
    double right = 0;

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
        hopper.setPower(opMode.gamepad2.right_stick_y * hopperSpd);
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
        lfWheel.setPower(-((opMode.gamepad1.left_stick_y - opMode.gamepad1.right_stick_x)*drvTrnSpd));
        rfWheel.setPower(-((opMode.gamepad1.left_stick_y + opMode.gamepad1.right_stick_x)*drvTrnSpd));
        lbWheel.setPower(-((opMode.gamepad1.left_stick_y - opMode.gamepad1.right_stick_x)*drvTrnSpd));
        rbWheel.setPower(-((opMode.gamepad1.left_stick_y + opMode.gamepad1.right_stick_x)*drvTrnSpd));
        // Carousel spinner slow.
        if (opMode.gamepad1.left_bumper) {
            carouselSpinnerL.setPower(.5);
            carouselSpinnerR.setPower(-.5);
        }
        // Carousel spinner fast.
        else if (opMode.gamepad1.right_bumper) {
            carouselSpinnerL.setPower(.35);
            carouselSpinnerR.setPower(-.35);
        }
        // Carousel spinner off.
        else if(!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper) {
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);
        }
        // Press 'a' to reset our heading.
        if (opMode.gamepad1.a) {
            headingAdjustment = currentHeading;
        }
        // Move to the last rotation we pressed 'a' at. 'x' will turn left and 'b' will turn right.
        if (opMode.gamepad1.x) {
            TeleIMUTurn(12, "l", 270, .3f,2);
        } else if (opMode.gamepad1.b) {
            TeleIMUTurn(-12, "r", 270, .3f, 2);
        }

        // Speed variation for the drivetrain.
        if (opMode.gamepad1.dpad_up){
            upFlag = true;
        } else {
            upFlag = false;
            upPersistent = false;
        }
        if (upFlag && !upPersistent) {
            if (drvTrnSpd < 1){drvTrnSpd += .1;}
            upPersistent = true;
        }
        if (opMode.gamepad1.dpad_down){
            downFlag = true;
        } else {
            downFlag = false;
            downPersistent = false;
        }
        if (downFlag && !downPersistent) {
            if (drvTrnSpd > .1){drvTrnSpd -= .1;}
            downPersistent = true;
        }
    }

    // Encoder drive function. DISCLAIMER: I did this quickly, so it's really ugly. Don't do this next year.
    // Like seriously, get a junk drivetrain and program on it; it'll be better than this.
    public void EncoderDrive(double inToMove, double maxSpeedDistance, double minSpeed, float timeOutB) {
        // Resetting the encoders. Encoders are plugged into the lf and rf motors. I am just too lazy to switch the ports on the control and expansion hubs.
        rbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Resetting the encoder drive timeout clock.
        encoderDriveTimer.reset();

        left = lbWheel.getCurrentPosition();
        right = rbWheel.getCurrentPosition();
        // Initializing the app. in. reading
        currentPosInches = ((left + right)/2)/ticksPerInch;

        // The important part says to keep moving until the current pos is grater than the target pos.
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentPosInches) < Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOutB) {
            // Updating the left and right motor readings.
            left = lbWheel.getCurrentPosition();
            right = rbWheel.getCurrentPosition();
            // Updating current pos.
            currentPosInches = ((left + right)/2)/ticksPerInch;

            // Making division by zero impossible.
            if (left == 0) {
                left = 1;
            } else if (right == 0) {
                right = 1;
            }

            // Constantly updating the power to the motors based on how far we have to move. This same line is used in the IMUTurn function. You can easily create proportional control by switching out a few of these variables.
            double power = Math.max(Math.abs(currentPosInches-inToMove)/maxSpeedDistance, minSpeed);

            if (right > left) {
                // Calculating how much we are veering.
                veer = (Math.abs(right / left)) * .9;

                // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
                if (inToMove >= 0) {
                    lfWheel.setPower(power * veer);
                    lbWheel.setPower(power * veer);
                    rfWheel.setPower(power);
                    rbWheel.setPower(power);
                } else if (inToMove < 0) {
                    lfWheel.setPower(-power * veer);
                    lbWheel.setPower(-power * veer);
                    rfWheel.setPower(-power);
                    rbWheel.setPower(-power);
                }
            }
            if (left >= right) {
                // Calculating how much we are veering.
                veer = (Math.abs(left / right)) * .9;

                // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
                if (inToMove >= 0) {
                    lfWheel.setPower(power);
                    lbWheel.setPower(power);
                    rfWheel.setPower(power * veer);
                    rbWheel.setPower(power * veer);
                } else if (inToMove < 0) {
                    lfWheel.setPower(-power);
                    lbWheel.setPower(-power);
                    rfWheel.setPower(-power * veer);
                    rbWheel.setPower(-power * veer);
                }
            }
        }
        // Stopping the drivetrain after reahing the target.
        lfWheel.setPower(0);
        rfWheel.setPower(0);
        lbWheel.setPower(0);
        rbWheel.setPower(0);
    }

    void IMUTurn(float targetAngle, String leftOrRight, int maxSpeedAngle, float minSpeed, float timeOutA) {
        // Resetting our timer
        imuTimer.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& imuTimer.seconds() < timeOutA) {
            // Updating our current heading while turning
            // Since our control Hub is vertical, this records the y axis angle in the variable 'currentHeading'
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (angles.firstAngle);
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
                lfWheel.setPower(-power);
                lbWheel.setPower(-power);
                rfWheel.setPower(power);
                rbWheel.setPower(power);
            } else {
                // Actually applying the power
                lfWheel.setPower(power);
                lbWheel.setPower(power);
                rfWheel.setPower(-power);
                rbWheel.setPower(-power);
            }
        }
        // Stopping the motors once we completed our turn
        lfWheel.setPower(0);
        lbWheel.setPower(0);
        rfWheel.setPower(0);
        rbWheel.setPower(0);
    }

    // Reading the barcode.
    public void ColorSensorReadings() {
        if (basePosNum == 1 || basePosNum == 2) {
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
        } else if (basePosNum == 3 || basePosNum == 4) {
            // Read the barcode pos for half a second.
            colorSensorTimer.reset();
            while (colorSensorTimer.seconds() < .5 && ((LinearOpMode) opMode).opModeIsActive()) {
                lReading = ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.INCH);
                rReading = ((DistanceSensor) colorSensorR).getDistance(DistanceUnit.INCH);
            }

            if (lReading < 3) {
                BarcodePosition = 1;
            } else if (rReading < 3) {
                BarcodePosition = 0;
            } else {
                BarcodePosition = 2;
            }
        }
    }

    // Blue autonomous path. Start next to carousel.
    public void BlueOne() {
        EncoderDrive(-16.75,40,.4,3);
        ColorSensorReadings();
        opMode.telemetry.addData("Pos: ", BarcodePosition);
        opMode.telemetry.update();
        EncoderDrive(3,15,.4,3);
        IMUTurn(-88f,"r", 200, .35f, 2);
        EncoderDrive(-19,25,.4,4);
        IMUTurn(-4f,"l", 120, .3f, 3);
        EncoderDrive(5.75,12,.4,3);
        CarouselAuto();
        EncoderDrive(-33, 48, .4, 3);
        IMUTurn(82,"l", 200, .35f, 3);
        EncoderDrive(-28, 46, .4, 3);
        DeliverBlock();
        EncoderDrive(30, 52, .4, 3);
        IMUTurn(7, "r", 200, .35f, 3);
        EncoderDrive(12.5, 40, .4, 3);
        while (((LinearOpMode)opMode).opModeIsActive()) {
            Telemetry();
        }
    }

    // Red autonomous path. Start next to carousel.
    public void RedOne() {
        EncoderDrive(-16,40,.35,3);
        ColorSensorReadings();
        opMode.telemetry.addData("Pos: ", BarcodePosition);
        opMode.telemetry.update();
        EncoderDrive(3,15,.35,3);
        IMUTurn(86f,"l", 200, .35f, 2);
        EncoderDrive(-19,25,.35,3);
        IMUTurn(2f,"r", 200, .3f, 3);
        EncoderDrive(5,12,.35,3);
        CarouselAuto();
        EncoderDrive(-34, 48, .35, 3);
        IMUTurn(-87,"r", 200, .3f, 3);
        EncoderDrive(-27, 46, .38, 3);
        DeliverBlock();
        EncoderDrive(31, 52, .38, 3);
        IMUTurn(-4, "l", 200, .34f, 3);
        EncoderDrive(13, 40, .38, 3);
    }

    // Blue autonomous path. Start in front of shipping hub.
    public void BlueTwo() {
        EncoderDrive(-21, 40, .35, 3);
        DeliverBlock();
        EncoderDrive(5,14, .4, 3);
        IMUTurn(88, "l", 200, .3f, 3);
        EncoderDrive(-60, 30, 1, 5);
    }


    // Red autonomous path. Start in front of shipping hub.
    public void RedTwo() {
        EncoderDrive(-21, 40, .35, 3);
        DeliverBlock();
        EncoderDrive(4,14, .35, 3);
        IMUTurn(-88, "r", 200, .3f, 3);
        EncoderDrive(-60, 30, 1, 5);
    }

    // Deliver the duck in autonomous.
    public void CarouselAuto() {
        // Retrieve the alliance and start position from the autonomous class.
        basePosNum = BobAuto.posNum;
        if (basePosNum == 1 || basePosNum == 2) {
            // Slowly back into the carousel.
            lfWheel.setPower(.25);
            // Turn on the carousel spinner for 4.5 seconds.
            carouselSpinnerL.setPower(-.3);
            carouselSpinnerR.setPower(.3);
            motorTimer.reset();
            while (motorTimer.seconds() < 4.5 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            // Stop the motors.
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);
            lfWheel.setPower(0);
        } else if (basePosNum == 3 || basePosNum == 4) {
            // Ditto
            rfWheel.setPower(.275);
            carouselSpinnerL.setPower(-.3);
            carouselSpinnerR.setPower(.3);
            motorTimer.reset();
            while (motorTimer.seconds() < 4.5 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            carouselSpinnerL.setPower(0);
            carouselSpinnerR.setPower(0);
            rfWheel.setPower(0);
        }
    }

    public void DeliverBlock() {
        if (BarcodePosition == 2) {
            // Raise the lift for 2 seconds.
            motorTimer.reset();
            lift.setPower(-.6);
            while (motorTimer.seconds() < 2 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            // Spin the hopper for 1 second.
            lift.setPower(-.1);
            hopper.setPower(-.4);
            motorTimer.reset();
            while (motorTimer.seconds() < 1 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            hopper.setPower(0);
            // Lower the lift
            lift.setPower(.8);
            motorTimer.reset();
            while (motorTimer.seconds() < 1 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            // Stop the lift.
            lift.setPower(0);
        } else if (BarcodePosition == 1) {
            // Ditto
            motorTimer.reset();
            lift.setPower(-.6);
            while (motorTimer.seconds() < 1.35 && ((LinearOpMode) opMode).opModeIsActive()) {
            }
            lift.setPower(-.1);
            hopper.setPower(-.4);
            motorTimer.reset();
            while (motorTimer.seconds() < 1 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            hopper.setPower(0);
            lift.setPower(.6);
            motorTimer.reset();
            while (motorTimer.seconds() < .7 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            lift.setPower(0);
        } else if (BarcodePosition == 0) {
            motorTimer.reset();
            lift.setPower(-.6);
            while (motorTimer.seconds() < .425 && ((LinearOpMode) opMode).opModeIsActive()) {
            }

            lift.setPower(-.1);
            hopper.setPower(-.4);
            motorTimer.reset();
            while (motorTimer.seconds() < 1 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            hopper.setPower(0);
            lift.setPower(-.9);
            motorTimer.reset();
            while (motorTimer.seconds() < 1 && ((LinearOpMode) opMode).opModeIsActive()) {
            }

            lift.setPower(.9);
            motorTimer.reset();
            while (motorTimer.seconds() < 1 && ((LinearOpMode) opMode).opModeIsActive()) {
                Telemetry();
            }
            lift.setPower(0);
        }
    }

    // Telemetry.
    public void Telemetry() {
        // Initializing the approximate inch reading
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (angles.firstAngle);
        adjCurrentHeading = currentHeading - headingAdjustment;
        opMode.telemetry.addData("Heading: ", adjCurrentHeading);
        opMode.telemetry.addData("Drivetrain speed: ", drvTrnSpd);
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

    // IMU turn to use in teleop. See 'IMUTurn' for comments
    void TeleIMUTurn(float targetAngle, String leftOrRight, int maxSpeedAngle, float minSpeed, float timeOutA) {
        imuTimer.reset();
        while (imuTimer.seconds() < timeOutA) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (angles.firstAngle);
            adjCurrentHeading = (angles.firstAngle-headingAdjustment);
            float power = Math.max(Math.abs(adjCurrentHeading-targetAngle)/maxSpeedAngle, minSpeed);
            if (Math.abs(adjCurrentHeading) < Math.abs(targetAngle)) {
                if ((Math.abs(adjCurrentHeading) - Math.abs(targetAngle)) > -5) {
                    break;
                }
            }
            if (leftOrRight.equals("left") || leftOrRight.equals("l")){
                lfWheel.setPower(power);
                lbWheel.setPower(power);
                rfWheel.setPower(-power);
                rbWheel.setPower(-power);
            } else {
                lfWheel.setPower(-power);
                lbWheel.setPower(-power);
                rfWheel.setPower(power);
                rbWheel.setPower(power);
            }
            opMode.telemetry.addData("Heading: ", adjCurrentHeading);
            opMode.telemetry.update();
        }
        lfWheel.setPower(0);
        lbWheel.setPower(0);
        rfWheel.setPower(0);
        rbWheel.setPower(0);
    }
}