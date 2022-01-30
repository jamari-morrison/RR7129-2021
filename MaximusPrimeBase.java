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
    // IMU setup
    BNO055IMU imu;
    Orientation angles;
    // Color sensor setup
    NormalizedColorSensor leftColorSensor;
    NormalizedColorSensor rightColorSensor;
    NormalizedColorSensor collectionColorSensor;
    // Timers
    ElapsedTime genericTimer = new ElapsedTime();
    ElapsedTime imuTimer = new ElapsedTime();
    ElapsedTime encoderDriveTimer = new ElapsedTime();
                        /*          AUTONOMOUS AND TELEOP VARIABLES         */
    // Deliver block variables
    LiftAvailability liftAvailability = LiftAvailability.OFF;
    private enum LiftAvailability{
        LOW, RAISING, DELIVERING, LOWERING, OFF
    }
    int highGoalTicks = 4200;
    int middleGoalTicks = 2200;
    int lowGoalTicks = 1000;
    int restingPositionTicks = 100;
    int liftTargetHeightTicks = lowGoalTicks;
    // Enumerator for determining alliance and autonomous positions
    Alliance alliance = Alliance.BLUE1;
    private enum Alliance{
        RED1, RED2, BLUE1, BLUE2
    }
    float IMUReading = 0;
    boolean autoTurnEnabled = false;
                        /*                TELEOP VARIABLES               */
    // Variables used in determining drivetrain speed
    double drvTrnSpd = .75;
    boolean upFlag = false;
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    // Variables used in field centric driver code
    int count = 0;
    double[] angleTest = new double[10];
    double average;
    double correct;
    double STARTING_HEADING = 0;
                        /*             AUTONOMOUS VARIABLES             */
    double currentLinearPositionInInches = 0;
    double amountOfVeer = 1;
    double ticksPerInch = 42.8;
    int leftSideEncoderAverage = 0;
    int rightSideEncoderAverage = 0;
    int frontSideEncoderAverage = 0;
    int backSideEncoderAverage = 0;
    double leftDistanceSensorReading = 0;
    double rightDistanceSensorReading = 0;
    float finalEncoderDriveHeading = 0;
    AutonomousTargetLevel autonomousTargetLevel = AutonomousTargetLevel.LOW;
    private enum AutonomousTargetLevel {
        HIGH, MIDDLE, LOW
    }
    // Link classes and run the configuration function
    public MaximusPrimeBase(OpMode theOpMode){
        opMode = theOpMode;
        Configuration();
    }
                                            /*Teleop Functions*/
    public void OperatorControls() {                                                                // Operator Controls
        // Turn off the automatic the lift if the joystick is used
        if (opMode.gamepad2.left_stick_y > .5 || opMode.gamepad2.left_stick_y < -.5) {
            liftAvailability = LiftAvailability.OFF;
        }
        // Lift controls.
        if (liftAvailability == LiftAvailability.OFF) {
            liftM.setPower(-opMode.gamepad2.left_stick_y);
        }
        // Collection controls.
        collectionM.setPower(opMode.gamepad2.right_stick_y);
        // Open/close delivery servo                                                                                        HELP PLS
        if (opMode.gamepad2.y) {
            deliveryS.setPosition(100);
        } else if (opMode.gamepad2.x) {
            deliveryS.setPosition(0);
        }
    }
    
    public void DriverControls() {                                                                  // Driver controls
        TeleIMUTurn();
        // Duck delivery controls
        // Clockwise for blue
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            // Spin the carousel quickly if the left bumper is pressed
            if (opMode.gamepad1.left_bumper) {
                lSpinnerM.setPower(-.5);
                rSpinnerM.setPower(-.5);
            }
            // Spin the carousel slowly if the right bumper is pressed
            else if (opMode.gamepad1.right_bumper) {
                lSpinnerM.setPower(-.35);
                rSpinnerM.setPower(-.35);
            }
            // Turn it off if neither are pressed
            else {
                lSpinnerM.setPower(0);
                rSpinnerM.setPower(0);
            }
        }
        // Counter-clockwise for red
        else if (alliance == Alliance.RED1 || alliance == Alliance.RED2) {
            // Spin the carousel quickly if the left bumper is pressed
            if (opMode.gamepad1.left_bumper) {
                lSpinnerM.setPower(.5);
                rSpinnerM.setPower(.5);
            }
            // Spin the carousel slowly if the right bumper is pressed
            else if (opMode.gamepad1.right_bumper) {
                lSpinnerM.setPower(.35);
                rSpinnerM.setPower(.35);
            }
            // Turn it off if neither are pressed
            else {
                lSpinnerM.setPower(0);
                rSpinnerM.setPower(0);
            }
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
    public void DeliverBlockTele() {                                                                // Deliver block tele
        // Reset home encoder reading if dpad right is pressed
        if (opMode.gamepad2.dpad_right) {
            liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // Turn on automatic delivery and set target location to the low, middle or
        // high goals based on what dpad button is pressed
        if (opMode.gamepad2.dpad_down) {
            liftTargetHeightTicks = lowGoalTicks;
            liftAvailability = LiftAvailability.LOW;
        } else if (opMode.gamepad2.dpad_left) {
            liftTargetHeightTicks = middleGoalTicks;
            liftAvailability = LiftAvailability.LOW;
        } else if (opMode.gamepad2.dpad_up) {
            liftTargetHeightTicks = highGoalTicks;
            liftAvailability = LiftAvailability.LOW;
        }

        // State machine to deliver the freight without any loops
        switch (liftAvailability) {
                // If the lift is low/unactivated and one of the dpad
                // buttons is pressed, begin raising the lift
            case LOW:
                if (opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left
                        || opMode.gamepad2.dpad_up) {
                    liftM.setPower(1);
                    liftAvailability = LiftAvailability.RAISING;
                }
                break;
                // If the lift has reached it's target position, begin delivering the freight
            case RAISING:
                if (liftM.getCurrentPosition() > liftTargetHeightTicks) {
                    liftM.setPower(0);
                    deliveryS.setPosition(50);
                    genericTimer.reset();
                    liftAvailability = LiftAvailability.DELIVERING;
                }
                break;
                // After the servo has been open for two seconds,
                // close the delivery and lower the lift
            case DELIVERING:
                if (genericTimer.seconds() > 1 &&
                        liftAvailability == LiftAvailability.DELIVERING) {
                    deliveryS.setPosition(0);
                    liftM.setPower(-1);
                    liftAvailability = LiftAvailability.LOWERING;
                }
                break;
                // Once the lift reaches the low position, stop the motor
            // and allow for this switch statement to begin again
            case LOWERING:
                if (liftM.getCurrentPosition() < restingPositionTicks) {
                    liftM.setPower(0);
                    liftAvailability = LiftAvailability.LOW;
                }
                break;
        }
    }
    void TeleIMUTurn() {                                                                            // Teleop IMU turn
        // Update our current heading
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        IMUReading = (angles.firstAngle);
        // If the dpad is pressed, determine which way to turn
        if (opMode.gamepad1.dpad_right) {
            // Turn off the manual drivetrain movement
            autoTurnEnabled = true;
            // Determine our speed based on how far a way we are from the target
            float power = Math.max(Math.abs(IMUReading)/75, .2f);
            // If we are to the left of the target, turn right
            if (IMUReading > 2) {
                setDrivePowerSides(power, power);
            }
            // If we are to the right of the target, turn left
            else if (IMUReading < -2) {
                setDrivePowerSides(-power, -power);
            }
            // If we are within two degrees of the target, stop
            else {
                stopDrivetrain();
            }
        }
        // If the automatic turn button is not pressed, turn on the manual drivetrain
        else {
            autoTurnEnabled = false;
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
            else {
                average = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0]
                        + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8]
                        + angleTest[9])/10;
                if(average > angle){
                    correct = average - angle;
                    angle = angle + correct;
                }
                else if(angle > average){
                    correct = angle - average;
                    angle = angle - correct;
                }
                count = 0;
            }
        }
        if (!autoTurnEnabled) {
            lfDrivetrainM.setPower((((speed * -(Math.sin(angle)) + turnPower))) * drvTrnSpd);
            lbDrivetrainM.setPower((((speed * -(Math.cos(angle)) + turnPower))) * drvTrnSpd);
            rfDrivetrainM.setPower((((speed * (Math.cos(angle))) + turnPower)) * drvTrnSpd);
            rbDrivetrainM.setPower((((speed * (Math.sin(angle))) + turnPower)) * drvTrnSpd);
        }
    }
                                            /*Autonomous Functions*/
    public void EncoderDrive(double inToMove, double maxSpeedDistance,                              // Encoder drive
                             double minSpeed, float timeOut, int headingOffset) {
        // Reset the stall out timer
        encoderDriveTimer.reset();
        // Reset the encoders before moving
        ResetEncoders();
        // Find the average of the encoders on the left side of the drivetrain
        leftSideEncoderAverage = (((lbDrivetrainM.getCurrentPosition()) +
                (lfDrivetrainM.getCurrentPosition()))/2);
        // Find the average of the encoders on the right side of the drivetrain
        rightSideEncoderAverage = -(((rbDrivetrainM.getCurrentPosition()) +
                (rfDrivetrainM.getCurrentPosition()))/2);
        // We divide a number by this variable, so this make division by zero impossible
        if (leftSideEncoderAverage == 0) {
            leftSideEncoderAverage = 1;
        }
        // We divide a number by this variable, so this make division by zero impossible
        if (rightSideEncoderAverage == 0) {
            rightSideEncoderAverage = 1;
        }
        // Find the average of the entire drivetrain
        currentLinearPositionInInches = ((leftSideEncoderAverage + rightSideEncoderAverage)/2)/ticksPerInch;

        // The important part says to keep moving until the
        // current position is grater than the target position
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentLinearPositionInInches) <
                Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOut) {
            // Find the average of the encoders on the left side of the drivetrain
            leftSideEncoderAverage = (((lbDrivetrainM.getCurrentPosition()) +
                    (lfDrivetrainM.getCurrentPosition()))/2);
            // Find the average of the encoders on the right side of the drivetrain
            rightSideEncoderAverage = -(((rbDrivetrainM.getCurrentPosition()) +
                    (rfDrivetrainM.getCurrentPosition()))/2);
            // Updating current pos.
            currentLinearPositionInInches = ((leftSideEncoderAverage + rightSideEncoderAverage)/2)/ticksPerInch;
            // We divide a number by this variable, so this make division by zero impossible
            if (leftSideEncoderAverage == 0) {
                leftSideEncoderAverage = 1;
            }
            // We divide a number by this variable, so this make division by zero impossible
            if (rightSideEncoderAverage == 0) {
                rightSideEncoderAverage = 1;
            }

            // Constantly updating the power to the motors based on how far we have to move.
            double power = Math.max(Math.abs(currentLinearPositionInInches -inToMove)/maxSpeedDistance, minSpeed);
            // Update our current heading
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            // Calculate how much we are veering using the Control Hub's IMU sensor
            amountOfVeer = Math.min(((IMUReading - headingOffset) / 25), .4);
            // If we are veering so that we are spinning clockwise,
            // subtract power from the appropriate motors
            if (amountOfVeer > 0) {
                // If we are moving forward, apply positive and
                // negative motor powers to the appropriate motors
                if (inToMove >= 0) {
                    setDrivePowerSides(power, (-power + amountOfVeer));
                }
                // If we are moving backwards, apply positive and
                // negative motor powers to the appropriate motors
                else if (inToMove < 0) {
                    setDrivePowerSides((-power + amountOfVeer), power);
                }
            }
            // If we are veering so that we are spinning counter-clockwise,
            // subtract power from the appropriate motors
            else if (amountOfVeer <= 0){
                // If we are moving forward, apply positive and
                // negative motor powers to the appropriate motors
                if (inToMove >= 0) {
                    setDrivePowerSides((power + amountOfVeer), -power);
                }
                // If we are moving backwards, apply positive and
                // negative motor powers to the appropriate motors
                else if (inToMove < 0) {
                    setDrivePowerSides(-power, (power + amountOfVeer));
                }
            }
            // Update telemetry
            Telemetry();
        }
        // Stopping the drivetrain after reaching the target.
        stopDrivetrain();
        // Reset encoders after movement
        ResetEncoders();
    }
    public void EncoderDriveSideways(double inToMove, double maxSpeedDistance,                      // Encoder Drive Sideways
                                     double minSpeed, float timeOut, int headingOffset) {
        // SEE ENCODERDRIVE FOR DOCUMENTATION
        encoderDriveTimer.reset();
        ResetEncoders();

        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        IMUReading = (angles.firstAngle);

        frontSideEncoderAverage = (((rfDrivetrainM.getCurrentPosition()) +
                (lfDrivetrainM.getCurrentPosition()))/2);
        rightSideEncoderAverage = -(((rbDrivetrainM.getCurrentPosition()) +
                (lbDrivetrainM.getCurrentPosition()))/2);

        if (frontSideEncoderAverage == 0) {
            frontSideEncoderAverage = 1;
        }
        if (backSideEncoderAverage == 0) {
            backSideEncoderAverage = 1;
        }
        currentLinearPositionInInches = ((frontSideEncoderAverage + backSideEncoderAverage)/2)/ticksPerInch;
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentLinearPositionInInches) <
                Math.abs(inToMove) && encoderDriveTimer.seconds() < timeOut) {
            frontSideEncoderAverage = (((rfDrivetrainM.getCurrentPosition()) +
                    (lfDrivetrainM.getCurrentPosition()))/2);
            rightSideEncoderAverage = -(((rbDrivetrainM.getCurrentPosition()) +
                    (lbDrivetrainM.getCurrentPosition()))/2);
            currentLinearPositionInInches = ((frontSideEncoderAverage + backSideEncoderAverage)/2)/ticksPerInch;
            if (frontSideEncoderAverage == 0) {
                frontSideEncoderAverage = 1;
            }
            if (backSideEncoderAverage == 0) {
                backSideEncoderAverage = 1;
            }
            double power = Math.max(Math.abs(currentLinearPositionInInches -inToMove)/maxSpeedDistance, minSpeed);
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            amountOfVeer = Math.min(((IMUReading - headingOffset) / 15), .4);
            if (amountOfVeer > 0) {
                if (inToMove >= 0) {
                    lfDrivetrainM.setPower(power);
                    lbDrivetrainM.setPower(-power + amountOfVeer);
                    rfDrivetrainM.setPower(power);
                    rbDrivetrainM.setPower(-power + amountOfVeer);
                } else if (inToMove < 0) {
                    lfDrivetrainM.setPower(-power + amountOfVeer);
                    lbDrivetrainM.setPower(power);
                    rfDrivetrainM.setPower(-power + amountOfVeer);
                    rbDrivetrainM.setPower(power);
                }
            } else if (amountOfVeer <= 0){
                if (inToMove >= 0) {
                    lfDrivetrainM.setPower(power - amountOfVeer);
                    lbDrivetrainM.setPower(-power);
                    rfDrivetrainM.setPower(power);
                    rbDrivetrainM.setPower(-power + amountOfVeer);
                } else if (inToMove < 0) {
                    lfDrivetrainM.setPower(-power + amountOfVeer);
                    lbDrivetrainM.setPower(power);
                    rfDrivetrainM.setPower(-power + amountOfVeer);
                    rbDrivetrainM.setPower(power);
                }
            }
            Telemetry();
        }
        stopDrivetrain();
        ResetEncoders();
    }
    void IMUTurn(float targetAngle, String leftOrRight,                                             // IMU Turn
                 float minSpeed, float timeOut) {
        // Reset the stall out timer
        imuTimer.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& imuTimer.seconds() < timeOut) {
            // Update our current heading while turning
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            // Calculating the motor powers based on how far away we are form the target angle
            float power = Math.max(Math.abs(IMUReading -targetAngle)/270, minSpeed);
            // If we are close enough to our target angle, break out of the while loop
            if (Math.abs(IMUReading) < Math.abs(targetAngle)) {
                if ((Math.abs(IMUReading) - Math.abs(targetAngle)) > -2){
                    break;
                }
            }
            // If we are turning left, apply positive and negative powers to the appropriate motors
            if (leftOrRight.equals("left") || leftOrRight.equals("l")){
                // Actually applying the power
                setDrivePowerSides(-power, -power);
            }
            // If we are turning right, apply positive and negative powers to the appropriate motors
            else {
                // Actually applying the power
                setDrivePowerSides(power, power);
            }
        }
        // Stopping the motors once we completed our turn
        stopDrivetrain();
    }
    public void RedOne() {                                                                          // Red One
        // Move backward to barcode
        EncoderDrive(-16,25,.3,2,0);
        // Read the Shipping Element's position
        ColorSensorReadings();
        // Move away from the barcode
        EncoderDrive(2,25,.3,2,0);
        // Strafe to the wall
        EncoderDriveSideways(18, 35,
                .2, 2, 0);
        // Move forward to carousel
        EncoderDrive(6,27,.3,2,0);
        // Deliver the duck
        CarouselAuto();
        // Move backward past Storage Unit
        EncoderDrive(-41, 49, .2, 2 , 2);
        // Strafe away from the wall
        EncoderDriveSideways(-5, 12,
                .2, 2, 0);
        // Turn towards Alliance Shipping Hub
        IMUTurn(-90, "r",.1f, 3);
        // Move forward to Alliance Shipping Hub
        EncoderDrive(-25, 37, .2, 2, -88);
        // Deliver the Pre-Loaded Box leaving the lift raised
        DeliverBlock();
        // Back up near the wall
        EncoderDrive(28, 37, .2, 2, -90);
        // Turn to be parallel with the wall
        IMUTurn(1, "l",.2f, 3);
        // Strafe to the wall
        EncoderDriveSideways(6, 11,
                .2, 2, 0);
        // Move forward into Storage Unit
        EncoderDrive(12, 22, .2, 2,0);
        // Lower the lift
        liftM.setPower(-1);
        while (liftM.getCurrentPosition() > 100) {
            Telemetry();
        }
        liftM.setPower(0);
    }
    public void BlueOne() {                                                                         // Blue One
        // Move backward to barcode
        EncoderDrive(-16,25,.3,2,0);
        // Read the Shipping Element's position
        ColorSensorReadings();
        // Move away from the barcode
        EncoderDrive(2,25,.3,2,0);
        // Strafe to the wall
        EncoderDriveSideways(-18, 35,
                .2, 2, 0);
        // Move forward to carousel
        EncoderDrive(6,27,.3,2,0);
        // Deliver the duck
        CarouselAuto();
        // Move backward past Storage Unit
        EncoderDrive(-41, 41, .2, 2 , -2);
        // Strafe away from the wall
        EncoderDriveSideways(5, 10,
                .2, 2, 0);
        // Turn towards Alliance Shipping Hub
        IMUTurn(90, "l",.2f, 3);
        // Move forward to Alliance Shipping Hub
        EncoderDrive(-25, 37, .2, 2, 90);
        // Deliver the Pre-Loaded Box leaving the lift raised
        DeliverBlock();
        // Back up near the wall
        EncoderDrive(28, 37, .2, 2, 90);
        // Turn to be parallel with the wall
        IMUTurn(-1, "r",.2f, 3);
        // Strafe to the wall
        EncoderDriveSideways(-6, 11,
                .2, 2, 0);
        // Move forward into Storage Unit
        EncoderDrive(12, 22, .2, 2,0);
        // Lower the lift
        liftM.setPower(-1);
        while (liftM.getCurrentPosition() > 100) {
            Telemetry();
        }
        liftM.setPower(0);
    }
    public void RedTwo() {                                                                          // Red Two
        // Move backward to barcode
        EncoderDrive(-16, 25, .2, 2, 0);
        // Read the Shipping Element's position
        ColorSensorReadings();
        // Move away from the barcode
        EncoderDrive(2, 25, .2, 2, 0);
        // Strafe to the front of the Alliance Shipping Hub
        EncoderDriveSideways(9, 34,
                .2, 2, 0);
        // Move backward to the Alliance Shipping Hub
        EncoderDrive(-12, 12, .2, 2, 0);
        // Deliver the Pre-Loaded freight
        DeliverBlock();
        // Move away from the Alliance Shipping Hub
        EncoderDrive(8, 15, .2,2,0);
        // Lower the lift
        liftM.setPower(-1);
        while (liftM.getCurrentPosition() > 100) {
            Telemetry();
        }
        liftM.setPower(0);
        // Turn parallel to the wall
        IMUTurn(90, "l",.1f, 3);
        // Strafe into the wall
        EncoderDriveSideways(9, 13,
                .2, 2, 90);
        // Move into the Warehouse
        EncoderDrive(52, 48, .4, 4, 88);
        // Strafe away from the wall
        EncoderDriveSideways(-7, 19,
                .2, 2, 90);
        // Turn perpendicular to the wall to set up the field centric driver code
        IMUTurn(-1, "r",.1f, 2);
        // Move out of the way if our alliance partners want to park in the warehouse
        EncoderDrive(-10, 28, .2, 2, 0);
    }
    public void BlueTwo() {                                                                         // Blue Two
        // Move backward to barcode
        EncoderDrive(-16, 25, .2, 2, 0);
        // Read the Shipping Element's position
        ColorSensorReadings();
        // Move away from the barcode
        EncoderDrive(2, 25, .2, 2, 0);
        // Strafe to the front of the Alliance Shipping Hub
        EncoderDriveSideways(-13, 34,
                .2, 2, 0);
        // Move backward to the Alliance Shipping Hub
        EncoderDrive(-9, 12, .2, 2, 0);
        // Deliver the Pre-Loaded freight
        DeliverBlock();
        // Move away from the Alliance Shipping Hub
        EncoderDrive(8, 15, .2,2,0);
        // Lower the lift
        liftM.setPower(-1);
        while (liftM.getCurrentPosition() > 100) {
            Telemetry();
        }
        liftM.setPower(0);
        // Turn parallel to the wall
        IMUTurn(-90, "r",.1f, 3);
        // Strafe into the wall
        EncoderDriveSideways(-9, 13,
                .2, 2, -90);
        // Move into the Warehouse
        EncoderDrive(52, 48, .4, 4, -88);
        // Strafe away from the wall
        EncoderDriveSideways(7, 19,
                .2, 2, -90);
        // Turn perpendicular to the wall to set up the field centric driver code
        IMUTurn(-1, "l",.1f, 2);
        // Move out of the way if our alliance partners want to park in the warehouse
        EncoderDrive(-10, 28, .2, 2, 0);
    }
    public void CarouselAuto() {                                                                    // Carousel Auto
        // If we are red
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            // Slowly move into the carousel
            lfDrivetrainM.setPower(.1);
            lfDrivetrainM.setPower(.1);
            // Turn on the carousel spinner for 4.5 seconds
            lSpinnerM.setPower(.35);
            rSpinnerM.setPower(.35);
            Sleep(4500);
            // Stop the motors
            lSpinnerM.setPower(0);
            rSpinnerM.setPower(0);
            lfDrivetrainM.setPower(0);
            lbDrivetrainM.setPower(0);
        }
        // If we are blue
        else if (alliance == Alliance.RED1 || alliance == Alliance.RED2) {
            // Slowly move into the carousel
            rfDrivetrainM.setPower(-.1);
            rbDrivetrainM.setPower(-.1);
            // Turn on the carousel spinner for 4.5 seconds
            lSpinnerM.setPower(.35);
            rSpinnerM.setPower(.35);
            Sleep(4500);
            // Stop the motors
            lSpinnerM.setPower(0);
            rSpinnerM.setPower(0);
            rfDrivetrainM.setPower(0);
        }
    }
    public void DeliverBlock() {                                                                    // Deliver Block
        // Reset the lift's home position
        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // If we should deliver into the high goal
        if (autonomousTargetLevel == AutonomousTargetLevel.HIGH) {
            // Raise the lift.
            liftM.setPower(1);
            while (liftM.getCurrentPosition() < 4200) {
                Telemetry();
            }
            liftM.setPower(0);
            // Deliver block
            deliveryS.setPosition(100);
            deliveryS.setPosition(0);
            Sleep(200);
            deliveryS.setPosition(100);
            Sleep(1000);
            deliveryS.setPosition(0);
            // Stop the lift
            liftM.setPower(0);
        }
        // If we should deliver into the high goal
        else if (autonomousTargetLevel == AutonomousTargetLevel.MIDDLE) {
            // Raise the lift
            liftM.setPower(1);
            while (liftM.getCurrentPosition() < 2200) {
                Telemetry();
            }
            liftM.setPower(0);
            // Deliver block
            deliveryS.setPosition(100);
            deliveryS.setPosition(0);
            Sleep(200);
            deliveryS.setPosition(100);
            Sleep(200);
            deliveryS.setPosition(0);
            // Stop the lift
            liftM.setPower(0);
        }
        // If we should deliver into the high goal
        else if (autonomousTargetLevel == AutonomousTargetLevel.LOW) {
            // Raise the lift
            liftM.setPower(1);
            while (liftM.getCurrentPosition() < 1000) {
                Telemetry();
            }
            liftM.setPower(0);
            // Deliver block
            deliveryS.setPosition(100);
            deliveryS.setPosition(0);
            Sleep(200);
            deliveryS.setPosition(100);
            Sleep(200);
            deliveryS.setPosition(0);
            // Stop the lift
            liftM.setPower(0);
        }
    }
    // Reading the barcode.
    public void ColorSensorReadings() {                                                             // Color Sensor Readings
        // If we are blue
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            // Read the barcode pos for half a second.
            genericTimer.reset();
            while (genericTimer.seconds() < .5 && ((LinearOpMode) opMode).opModeIsActive()) {
                UpdateColorSensor();
            }
            // If the left distance sensor is triggered, set the
            // Shipping Element position to the middle goal
            if (leftDistanceSensorReading < 3) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            }
            // If the right distance sensor is triggered, set the
            // Shipping Element position to the high goal
            else if (rightDistanceSensorReading < 3) {
                autonomousTargetLevel = AutonomousTargetLevel.HIGH;
            }
            // If neither sensors are triggered, set the Shipping Element position to the low goal
            else {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
        }
        // If we are red
        else if (alliance == Alliance.RED1 || alliance == Alliance.RED2) {
            // Read the barcode pos for half a second.
            genericTimer.reset();
            while (genericTimer.seconds() < .5 && ((LinearOpMode) opMode).opModeIsActive()) {
                UpdateColorSensor();
            }
            // If the left distance sensor is triggered, set the
            // Shipping Element position to the low goal
            if (leftDistanceSensorReading < 3) {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
            // If the right distance sensor is triggered, set the
            // Shipping Element position to the middle goal
            else if (rightDistanceSensorReading < 3) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            }
            // If neither sensors are triggered, set the Shipping Element position to the high goal
            else {
                autonomousTargetLevel = AutonomousTargetLevel.HIGH;
            }
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
        leftColorSensor = opMode.hardwareMap.
                get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor = opMode.hardwareMap.
                get(NormalizedColorSensor.class, "rightColorSensor");
        collectionColorSensor = opMode.hardwareMap.
                get(NormalizedColorSensor.class, "collectionColorSensor");
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
        leftDistanceSensorReading = ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM);
        rightDistanceSensorReading = ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM);
    }

    public void Telemetry() {                                                                       // Telemetry
        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        IMUReading = (angles.firstAngle);
        opMode.telemetry.addData("Left: ", leftDistanceSensorReading);
        opMode.telemetry.addData("Right: ", rightDistanceSensorReading);
        opMode.telemetry.addData("Heading: ", IMUReading);
        opMode.telemetry.addData("Heading off: ", finalEncoderDriveHeading);
        opMode.telemetry.addData("Drivetrain speed: ", drvTrnSpd);
        opMode.telemetry.update();
    }

    public void AllianceDetermination() {                                                           // Alliance determination
        // Choosing our starting position for autonomous and teleop.
        // X for Blue 1, A for Blue 2, etc.
        if (opMode.gamepad1.x) {
            alliance = Alliance.BLUE1;
        } else if (opMode.gamepad1.a) {
            alliance = Alliance.BLUE2;
        } else if (opMode.gamepad1.b) {
            alliance = Alliance.RED1;
        } else if (opMode.gamepad1.y) {
            alliance = Alliance.RED2;
        }
        // Telemeter the position currently selected
        opMode.telemetry.addData("Starting auto position", alliance);
        opMode.telemetry.update();
    }
    public void ResetEncoders() {
        // Function to reset encoders
        lfDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrivetrainM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrivetrainM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Autonomous() {
        // Play the autonomous based on what path was chosen
        if (alliance == Alliance.RED1) {
            RedOne();
        } else if (alliance == Alliance.BLUE1) {
            BlueOne();
        } else if (alliance == Alliance.RED2) {
            RedTwo();
        } else if (alliance == Alliance.BLUE2) {
            BlueTwo();
        }
    }
    boolean IsInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }
    public void Sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }

    public void setDrivePowerSides(double motorPowerL, double motorPowerR) {
        lfDrivetrainM.setPower(motorPowerL);
        lbDrivetrainM.setPower(motorPowerL);
        rfDrivetrainM.setPower(motorPowerR);
        rbDrivetrainM.setPower(motorPowerR);
    }

    public void stopDrivetrain() {
        lfDrivetrainM.setPower(0);
        lbDrivetrainM.setPower(0);
        rfDrivetrainM.setPower(0);
        rbDrivetrainM.setPower(0);
    }
}