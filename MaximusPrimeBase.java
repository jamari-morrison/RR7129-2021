package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class MaximusPrimeBase {
    OpMode opMode;

    // Motor declaration
    DcMotor collectionM, liftM, lSpinnerM, rSpinnerM,
            lfDrivetrainM, rfDrivetrainM, lbDrivetrainM, rbDrivetrainM;
    // Servo declaration
    Servo deliveryS;
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
    int currentLiftHeight = 0;              // The lift's current height
    int liftOffset = 0;                     // Variable to reset lift home
    double drvTrnSpd = .75;                 // Drivetrain speed
    boolean upFlag = false;                 // Variables used in determining drivetrain speed
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    double currentPosInches = 0;            // Variables used in encoder drive
    double veer = 1;
    double ticksPerInch = 42.8;
    int lfOffset = 0;
    int rfOffset = 0;
    int lbOffset = 0;
    int rbOffset = 0;
    int leftSideEncoder = 0;
    int rightSideEncoder = 0;
    int count = 0;                          // Variables used in field centric driver code
    double angleTest[] = new double[10];
    double sum;
    double correct;
    double STARTING_HEADING = 0;
    float currentHeading = 0;               // Reading from IMU
    boolean autoTurn = false;               // Check if we are automatically turning
    boolean isBlue = true;                  // Determines carousel direction

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
            liftM.setPower(opMode.gamepad2.left_stick_y);
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
        if (opMode.gamepad1.left_bumper) {
            lSpinnerM.setPower(.5);
            rSpinnerM.setPower(-.5);
        }
        // Spin the carousel slowly if the right bumper is pressed
        else if (opMode.gamepad1.right_bumper) {
            lSpinnerM.setPower(.35);
            rSpinnerM.setPower(-.35);
        }
        // Turn the carousel off if no bumpers are pressed
        else if(!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper) {
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
        opMode.telemetry.addData("Heading: ", currentHeading);
        opMode.telemetry.addData("Starting heading: ", STARTING_HEADING);
        opMode.telemetry.addData("Drivetrain speed: ", drvTrnSpd);
        opMode.telemetry.addData("Lift height: ", liftM.getCurrentPosition());
        opMode.telemetry.addData("Lift height (adjusted) : ", currentLiftHeight);
        opMode.telemetry.addData("LF", lfDrivetrainM.getCurrentPosition()-lfOffset);
        opMode.telemetry.addData("RF", rfDrivetrainM.getCurrentPosition()-rfOffset);
        opMode.telemetry.addData("LB", lbDrivetrainM.getCurrentPosition()-lbOffset);
        opMode.telemetry.addData("RB", rbDrivetrainM.getCurrentPosition()-rbOffset);
        opMode.telemetry.addData("LFo", lfOffset);
        opMode.telemetry.addData("RFo", rfOffset);
        opMode.telemetry.addData("LBo", lbOffset);
        opMode.telemetry.addData("RBo", rbOffset);
        opMode.telemetry.addData("Left: ", leftSideEncoder/ticksPerInch);
        opMode.telemetry.addData("Right: ", rightSideEncoder/ticksPerInch);
        opMode.telemetry.update();
    }

    public void DeliverBlockTele() {                                                                // Deliver block tele
        currentLiftHeight = liftM.getCurrentPosition() - liftOffset;
        if (opMode.gamepad2.dpad_right) {
            liftOffset = liftM.getCurrentPosition();
        }
        if (opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_up) {
            manualLift = false;
        }
        if (opMode.gamepad2.dpad_down) {
            liftTargetHeight = 100;
        } else if (opMode.gamepad2.dpad_left) {
            liftTargetHeight = 500;
        } else if (opMode.gamepad2.dpad_up) {
            liftTargetHeight = 1000;
        }
        // Raise the lift
        if (opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_up
                && liftAvailability.equals("Low")) {
            liftM.setPower(1);
            manualLift = false;
            liftAvailability = "Raising";
        }
        // Stop the lift at the correct level
        if (!manualLift && currentLiftHeight > liftTargetHeight
                && liftAvailability.equals("Raising")) {
            liftM.setPower(0);
            liftAvailability = "High";
        }
        // Open the lift
        if (!manualLift && liftAvailability.equals("High")) {
            deliveryS.setPosition(100);
            deliverBlockTimer.reset();
            liftAvailability = "Delivering";
        }
        // Close the lift and lower the lift after 2 seconds
        if (!manualLift && deliverBlockTimer.seconds() > 2
                && liftAvailability.equals("Delivering")) {
            deliveryS.setPosition(0);
            liftAvailability = ("Lowering");
            liftM.setPower(-1);
        }
        // Stop the lift at the correct level
        if (!manualLift && currentLiftHeight < 0 && liftAvailability.equals("Lowering")) {
            liftM.setPower(0);
            liftAvailability = "Low";
        }
    }

    void TeleIMUTurn() {
        imuTimer.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (angles.firstAngle);
        if (opMode.gamepad1.dpad_up) {
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

    public void TeleopAllianceDetermination() {                                                     // Teleop alliance determination
        if (opMode.gamepad1.x) {
            isBlue = false;
        } else if (opMode.gamepad1.b) {
            isBlue = true;
        }
        opMode.telemetry.addData("Press X for red and B for blue", "");
        opMode.telemetry.addData("Are we blue? ", isBlue);
        opMode.telemetry.update();
    }

    //                                  Autonomous Functions

    public void EncoderDrive(double inToMove, double maxSpeedDistance, double minSpeed, float timeOutB) {
        encoderDriveTimer.reset();
        lfOffset = lfDrivetrainM.getCurrentPosition();
        rfOffset = rfDrivetrainM.getCurrentPosition();
        lbOffset = lbDrivetrainM.getCurrentPosition();
        rbOffset = rbDrivetrainM.getCurrentPosition();

        leftSideEncoder = (((lbDrivetrainM.getCurrentPosition()-lbOffset) + (lfDrivetrainM.getCurrentPosition()-lfOffset))/2);
        rightSideEncoder = -(((rbDrivetrainM.getCurrentPosition()-rbOffset) + (rfDrivetrainM.getCurrentPosition()-rfOffset))/2);
        // Initializing the app. in. reading
        currentPosInches = ((leftSideEncoder + rightSideEncoder)/2)/ticksPerInch;

        // The important part says to keep moving until the current pos is grater than the target pos.
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentPosInches) < Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOutB) {
            // Updating the left and right motor readings.
            leftSideEncoder = (((lbDrivetrainM.getCurrentPosition()-lbOffset) + (lfDrivetrainM.getCurrentPosition()-lfOffset))/2);
            rightSideEncoder = -(((rbDrivetrainM.getCurrentPosition()-rbOffset) + (rfDrivetrainM.getCurrentPosition()-rfOffset))/2);
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

            if (rightSideEncoder > leftSideEncoder) {
                // Calculating how much we are veering.
                veer = (Math.abs(rightSideEncoder / leftSideEncoder)) * .9;

                // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
                if (inToMove >= 0) {
                    lfDrivetrainM.setPower(power );
                    lbDrivetrainM.setPower(power);
                    rfDrivetrainM.setPower(-power);
                    rbDrivetrainM.setPower(-power);
                } else if (inToMove < 0) {
                    lfDrivetrainM.setPower(-power);
                    lbDrivetrainM.setPower(-power);
                    rfDrivetrainM.setPower(power);
                    rbDrivetrainM.setPower(power);
                }
            }
            if (leftSideEncoder >= rightSideEncoder) {
                // Calculating how much we are veering.
                veer = (Math.abs(leftSideEncoder / rightSideEncoder)) * .9;

                // Assigning the motor powers. "veer" is a variable accounting for the right side of the drivetrain being faster than the left.
                if (inToMove >= 0) {
                    lfDrivetrainM.setPower(power);
                    lbDrivetrainM.setPower(power);
                    rfDrivetrainM.setPower(-power );
                    rbDrivetrainM.setPower(-power );
                } else if (inToMove < 0) {
                    lfDrivetrainM.setPower(-power);
                    lbDrivetrainM.setPower(-power);
                    rfDrivetrainM.setPower(power );
                    rbDrivetrainM.setPower(power );
                }
            }
        }
        // Stopping the drivetrain after reaching the target.
        lfDrivetrainM.setPower(0);
        rfDrivetrainM.setPower(0);
        lbDrivetrainM.setPower(0);
        rbDrivetrainM.setPower(0);
        lfOffset = lfDrivetrainM.getCurrentPosition();
        rfOffset = rfDrivetrainM.getCurrentPosition();
        lbOffset = lbDrivetrainM.getCurrentPosition();
        rbOffset = rbDrivetrainM.getCurrentPosition();
    }

    public void ResetEncoders() {
        lfOffset = lfDrivetrainM.getCurrentPosition();
        rfOffset = rfDrivetrainM.getCurrentPosition();
        lbOffset = lbDrivetrainM.getCurrentPosition();
        rbOffset = rbDrivetrainM.getCurrentPosition();
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
                if ((Math.abs(currentHeading) - Math.abs(targetAngle)) > -.5){
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

    public void BlueOne() {
        EncoderDrive(30,20,.3,2);
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
}