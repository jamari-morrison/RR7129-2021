package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class MaximusPrimeBase {
    OpMode opMode;

    // Motor declaration
    DcMotor collectionM, liftM, lSpinnerM, rSpinnerM,
            lfDrivetrainM, rfDrivetrainM, lbDrivetrainM, rbDrivetrainM;
    // Servo declaration
    Servo deliveryS;

    NormalizedColorSensor leftColorSensor;
    NormalizedColorSensor rightColorSensor;
    // Timers
    ElapsedTime deliverBlockTimer = new ElapsedTime();
    ElapsedTime imuTimer = new ElapsedTime();
    ElapsedTime encoderDriveTimer = new ElapsedTime();
    // IMU declaration
    BNO055IMU imu;
    Orientation angles;
    String liftAvailability = "Low";        // Lift state
    boolean manualLift = false;             // Manual or auto lift
    int liftTargetHeight = 1000;            // Height of the target level on shipping hub in ticks
    double drvTrnSpd = .75;                 // Drivetrain speed
    boolean upFlag = false;                 // Variables used in determining drivetrain speed
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    double currentPosInches = 0;            // Variables used in encoder drive
    double veer = 1;
    double ticksPerInch = 42.8;
    int leftSideEncoder = 0;
    int rightSideEncoder = 0;
    int count = 0;                          // Variables used in field centric driver code
    double angleTest[] = new double[10];
    double sum;
    double correct;
    double STARTING_HEADING = 0;
    double leftDistance = 0;
    double rightDistance = 0;
    float currentHeading = 0;               // Reading from IMU
    float headingOffset = 0;
    boolean autoTurn = false;               // Check if we are automatically turning
    int alliance = 3;                  // Determines carousel direction
    boolean a = false;
    boolean b = false;
    boolean c = false;

    public MaximusPrimeBase(OpMode theOpMode){
        opMode = theOpMode;
        Configuration();
    }


    //                                  Teleop functions
    public void OperatorControls() {                                                                // Operator controls
        // Toggle auto lift
        if (opMode.gamepad2.left_stick_y > .5 ||
                opMode.gamepad2.left_stick_y < -.5) {
            manualLift = true;
            liftAvailability = "Low";
        } else if (opMode.gamepad2.b) {
            manualLift = false;
        }
        // Lift controls.
        if (manualLift) {
            liftM.setPower(-opMode.gamepad2.left_stick_y);
        }
        // Collection controls.
        collectionM.setPower(opMode.gamepad2.right_stick_y);
        // Open/close delivery servo
        if (opMode.gamepad2.y) {
            deliveryS.setPosition(100);
        } else if (opMode.gamepad2.x) {
            deliveryS.setPosition(0);
        }
    }

    public void DriverControls() {                                                                  // Driver controls
        TeleIMUTurn();
        // Spin the carousel quickly if the left bumper is pressed
        if (alliance == 1 || alliance == 2) {
            if (opMode.gamepad1.left_bumper) {
                lSpinnerM.setPower(-.5);
                rSpinnerM.setPower(-.5);
            }
            // Spin the carousel slowly if the right bumper is pressed
            else if (opMode.gamepad1.right_bumper) {
                lSpinnerM.setPower(-.35);
                rSpinnerM.setPower(-.35);
            } else {
                lSpinnerM.setPower(0);
                rSpinnerM.setPower(0);
            }
        } else if (alliance == 3 || alliance == 4) {
            if (opMode.gamepad1.left_bumper) {
                lSpinnerM.setPower(.5);
                rSpinnerM.setPower(.5);
            }
            // Spin the carousel slowly if the right bumper is pressed
            else if (opMode.gamepad1.right_bumper) {
                lSpinnerM.setPower(.35);
                rSpinnerM.setPower(.35);
            } else {
                lSpinnerM.setPower(0);
                rSpinnerM.setPower(0);
            }
        } else if (!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper) {
            lSpinnerM.setPower(0);
            rSpinnerM.setPower(0);
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

    public void Telemetry() {                                                                       // Telemetry
        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (angles.firstAngle);
        opMode.telemetry.addData("Left: ", leftDistance);
        opMode.telemetry.addData("Right: ", rightDistance);
        opMode.telemetry.addData("Heading: ", currentHeading);
        opMode.telemetry.addData("Heading off: ", headingOffset);
        opMode.telemetry.addData("Drivetrain speed: ", drvTrnSpd);
        opMode.telemetry.update();
    }

    public void DeliverBlockTele() {                                                                // Deliver block tele
        if (opMode.gamepad2.dpad_right) {
            liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_up) {
            manualLift = false;
        }
        if (opMode.gamepad2.dpad_down) {
            liftTargetHeight = 1000;
        } else if (opMode.gamepad2.dpad_left) {
            liftTargetHeight = 2200;
        } else if (opMode.gamepad2.dpad_up) {
            liftTargetHeight = 4200;
        }
        // Raise the lift
        if (opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_up
                && liftAvailability.equals("Low")) {
            liftM.setPower(1);
            manualLift = false;
            liftAvailability = "Raising";
        }
        // Stop the lift at the correct level
        if (!manualLift && liftM.getCurrentPosition() > liftTargetHeight
                && liftAvailability.equals("Raising")) {
            liftM.setPower(0);
            liftAvailability = "High";
        }
        // Open the lift
        if (!manualLift && liftAvailability.equals("High")) {
            deliveryS.setPosition(50);
            deliverBlockTimer.reset();
            liftAvailability = "Delivering";
        }
        // Close the lift and lower the lift after 2 seconds
        if (!manualLift && deliverBlockTimer.seconds() > 1
                && liftAvailability.equals("Delivering")) {
            deliveryS.setPosition(0);
            liftAvailability = ("Lowering");
            liftM.setPower(-1);
        }
        // Stop the lift at the correct level
        if (!manualLift && liftM.getCurrentPosition() < 0 && liftAvailability.equals("Lowering")) {
            liftM.setPower(0);
            liftAvailability = "Low";
        }
    }

    void TeleIMUTurn() {                                                                            // Teleop IMU turn
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (angles.firstAngle);
        if (opMode.gamepad1.dpad_right) {
            autoTurn = true;
            float power = Math.max(Math.abs(currentHeading)/75, .2f);
            if (currentHeading > 2) {
                lfDrivetrainM.setPower(power);
                lbDrivetrainM.setPower(power);
                rfDrivetrainM.setPower(power);
                rbDrivetrainM.setPower(power);
            } else if (currentHeading < -2) {
                lfDrivetrainM.setPower(-power);
                lbDrivetrainM.setPower(-power);
                rfDrivetrainM.setPower(-power);
                rbDrivetrainM.setPower(-power);
            } else {
                lfDrivetrainM.setPower(0);
                lbDrivetrainM.setPower(0);
                rfDrivetrainM.setPower(0);
                rbDrivetrainM.setPower(0);
            }
        } else {
            autoTurn = false;
        }
    }

    public void UpdateDriveTrain() {                                                                // Field-centric driver code
        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double speed = Math.hypot
                (opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2
                (opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4);
        angle += angles.firstAngle - (Math.PI)/2 + STARTING_HEADING;
        double turnPower = opMode.gamepad1.right_stick_x;
        if(turnPower == 0){
            if (count < 10) {
                angleTest[count] = angle;
                angleTest[count] = angle;
                count++;
            }
            else if (count >= 10){
                sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0]
                        + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8]
                        + angleTest[9])/10;
                if(sum > angle){
                    correct = sum - angle;
                    angle = angle + correct;
                }
                else if(angle > sum){
                    correct = angle - sum;
                    angle = angle - correct;
                }
                count = 0;
            }
        }
        if (!autoTurn) {
            lfDrivetrainM.setPower((((speed * -(Math.sin(angle)) + turnPower))) * drvTrnSpd);
            lbDrivetrainM.setPower((((speed * -(Math.cos(angle)) + turnPower))) * drvTrnSpd);
            rfDrivetrainM.setPower((((speed * (Math.cos(angle))) + turnPower)) * drvTrnSpd);
            rbDrivetrainM.setPower((((speed * (Math.sin(angle))) + turnPower)) * drvTrnSpd);
        }
    }

    public void ResetHeading(){                                                                     // Reset heading
        if (opMode.gamepad1.b){
            angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (angles.firstAngle);
            STARTING_HEADING = angles.firstAngle;
        }
    }

    public void AllianceDetermination() {                                                     // Teleop alliance determination
        if (opMode.gamepad1.x) {
            alliance = 1;
        } else if (opMode.gamepad1.a) {
            alliance = 2;
        } else if (opMode.gamepad1.b) {
            alliance = 3;
        } else if (opMode.gamepad1.y) {
            alliance = 4;
        }
        if (alliance == 1) {
            opMode.telemetry.addData("Blue 1 ", "");
            opMode.telemetry.update();
        } else if (alliance == 2) {
            opMode.telemetry.addData("Blue 2 ", "");
            opMode.telemetry.update();
        } else if (alliance == 3) {
            opMode.telemetry.addData("Red 1 ", "");
            opMode.telemetry.update();
        } else if (alliance == 4) {
            opMode.telemetry.addData("Red 2 ", "");
            opMode.telemetry.update();
        }
    }

    //                                  Autonomous Functions

    public void EncoderDrive(double inToMove, double maxSpeedDistance, double minSpeed, float timeOutB, int headingOffset) {
        encoderDriveTimer.reset();
        ResetEncoders();

        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (angles.firstAngle);

        leftSideEncoder = (((lbDrivetrainM.getCurrentPosition()) + (lfDrivetrainM.getCurrentPosition()))/2);
        rightSideEncoder = -(((rbDrivetrainM.getCurrentPosition()) + (rfDrivetrainM.getCurrentPosition()))/2);
        // Initializing the app. in. reading
        if (leftSideEncoder == 0) {
            leftSideEncoder = 1;
        }
        if (rightSideEncoder == 0) {
            rightSideEncoder = 1;
        }
        currentPosInches = ((leftSideEncoder + rightSideEncoder)/2)/ticksPerInch;

        // The important part says to keep moving until the current pos is grater than the target pos.
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentPosInches) < Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOutB) {
            // Updating the left and right motor readings.
            leftSideEncoder = (((lbDrivetrainM.getCurrentPosition()) + (lfDrivetrainM.getCurrentPosition()))/2);
            rightSideEncoder = -(((rbDrivetrainM.getCurrentPosition()) + (rfDrivetrainM.getCurrentPosition()))/2);
            // Updating current pos.
            currentPosInches = ((leftSideEncoder + rightSideEncoder)/2)/ticksPerInch;

            // Making division by zero impossible.
            if (leftSideEncoder == 0) {
                leftSideEncoder = 1;
            }
            if (rightSideEncoder == 0) {
                rightSideEncoder = 1;
            }

            // Constantly updating the power to the motors based on how far we have to move. This same line is used in the IMUTurn function. You can easily create proportional control by switching out a few of these variables.
            double power = Math.max(Math.abs(currentPosInches-inToMove)/maxSpeedDistance, minSpeed);

            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (angles.firstAngle);
            // Calculating how much we are veering.
            veer = Math.min(((currentHeading - headingOffset) / 25), .4);
            if (veer > 0) {

                // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
                if (inToMove >= 0) {
                    lfDrivetrainM.setPower(power);
                    lbDrivetrainM.setPower(power);
                    rfDrivetrainM.setPower(-power + veer);
                    rbDrivetrainM.setPower(-power + veer);
                } else if (inToMove < 0) {
                    lfDrivetrainM.setPower(-power +  veer);
                    lbDrivetrainM.setPower(-power +  veer);
                    rfDrivetrainM.setPower(power);
                    rbDrivetrainM.setPower(power);
                }
            } else if (veer <= 0){

                // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
                if (inToMove >= 0) {
                    lfDrivetrainM.setPower(power + veer);
                    lbDrivetrainM.setPower(power + veer);
                    rfDrivetrainM.setPower(-power);
                    rbDrivetrainM.setPower(-power);
                } else if (inToMove < 0) {
                    lfDrivetrainM.setPower(-power);
                    lbDrivetrainM.setPower(-power);
                    rfDrivetrainM.setPower(power +  veer);
                    rbDrivetrainM.setPower(power +  veer);
                }
            }
            Telemetry();
        }
        // Stopping the drivetrain after reaching the target.
        lfDrivetrainM.setPower(0);
        rfDrivetrainM.setPower(0);
        lbDrivetrainM.setPower(0);
        rbDrivetrainM.setPower(0);
        ResetEncoders();
    }

    public void ResetEncoders() {
        lfDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void IMUTurn(float targetAngle, String leftOrRight, int maxSpeedAngle, float minSpeed, float timeOutA) {
        // Resetting our timer
        imuTimer.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& imuTimer.seconds() < timeOutA) {
            // Updating our current heading while turning
            // This records the z axis angle in the variable 'currentHeading'
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (angles.firstAngle);
            // Calculating the motor power. 'minSpeed' is the slowest speed to turn at.
            // 'maxSpeedAngle' is how many degrees of the angle it will turn without slowing down
            float power = Math.max(Math.abs(currentHeading-targetAngle)/maxSpeedAngle, minSpeed);

            if (Math.abs(currentHeading) < Math.abs(targetAngle)) {
                if ((Math.abs(currentHeading) - Math.abs(targetAngle)) > -2){
                    break;
                }
            }

            if (leftOrRight.equals("left") || leftOrRight.equals("l")){
                // Actually applying the power
                lfDrivetrainM.setPower(-power);
                lbDrivetrainM.setPower(-power);
                rfDrivetrainM.setPower(-power);
                rbDrivetrainM.setPower(-power);
            } else {
                // Actually applying the power
                lfDrivetrainM.setPower(power);
                lbDrivetrainM.setPower(power);
                rfDrivetrainM.setPower(power);
                rbDrivetrainM.setPower(power);
            }
        }
        // Stopping the motors once we completed our turn
        lfDrivetrainM.setPower(0);
        lbDrivetrainM.setPower(0);
        rfDrivetrainM.setPower(0);
        rbDrivetrainM.setPower(0);
    }

    public void RedOne() {
        EncoderDrive(-15,25,.3,2,0);
        Sleep(1000);
        EncoderDrive(2,25,.3,2,0);
        IMUTurn(90,"l",180,.15f,3);
        EncoderDrive(-27,28,.3,2,90);
        IMUTurn(2,"r",270,.1f,3);
        EncoderDrive(5,27,.3,2,0);
        CarouselAuto();
    }

    public void BlueOne() {

    }


    public void CarouselAuto() {
        // Retrieve the alliance and start position from the autonomous class.
        if (alliance == 1 || alliance == 2) {
            // Slowly back into the carousel.
            lfDrivetrainM.setPower(.1);
            lfDrivetrainM.setPower(.1);
            // Turn on the carousel spinner for 4.5 seconds.
            lSpinnerM.setPower(.35);
            rSpinnerM.setPower(.35);
            Sleep(4500);
            // Stop the motors.
            lSpinnerM.setPower(0);
            rSpinnerM.setPower(0);
            lfDrivetrainM.setPower(0);
            lbDrivetrainM.setPower(0);
        } else if (alliance == 3 || alliance == 4) {
            // Ditto
            rfDrivetrainM.setPower(-.1);
            rbDrivetrainM.setPower(-.1);
            lSpinnerM.setPower(.35);
            rSpinnerM.setPower(.35);
            Sleep(4500);
            lSpinnerM.setPower(0);
            rSpinnerM.setPower(0);
            rfDrivetrainM.setPower(0);
        }
    }

    boolean IsInitialized() {                                                                       // Is initialized
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }

    public void Sleep(long ms) {                                                                    // Sleep
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }

    public void Configuration() {                                                                   // Configuration
        collectionM = opMode.hardwareMap.dcMotor.get("collectionM");
        liftM = opMode.hardwareMap.dcMotor.get("liftM");
        lSpinnerM = opMode.hardwareMap.dcMotor.get("lSpinnerM");
        rSpinnerM = opMode.hardwareMap.dcMotor.get("rSpinnerM");
        lfDrivetrainM = opMode.hardwareMap.dcMotor.get("lfDrvtrnM");
        rfDrivetrainM = opMode.hardwareMap.dcMotor.get("rfDrvtrnM");
        lbDrivetrainM = opMode.hardwareMap.dcMotor.get("lbDrvtrnM");
        rbDrivetrainM = opMode.hardwareMap.dcMotor.get("rbDrvtrnM");
        this.deliveryS = opMode.hardwareMap.get(Servo.class, "deliveryS");
        leftColorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
        liftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrivetrainM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrivetrainM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrivetrainM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrivetrainM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void UpdateColorSensor() {
        leftDistance = ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM);
        rightDistance = ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM);
    }
}