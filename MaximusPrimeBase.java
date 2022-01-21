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
    DcMotor collectionM, liftM, lSpinnerM, rSpinnerM, lfDrivetrainM, rfDrivetrainM, lbDrivetrainM, rbDrivetrainM;
    // Servo declaration
    Servo deliveryS;
    // Timers
    ElapsedTime deliverBlockTimer = new ElapsedTime();
    ElapsedTime imuTimer = new ElapsedTime();
    // IMU declaration
    BNO055IMU imu;
    Orientation angles;
    String liftAvailability = "Low";        // Lift state
    boolean manualLift = false;             // Manual or auto lift
    int liftHeight = 1000;                  // Height of third level on shipping hub in ticks
    double drvTrnSpd = .75;                 // Drivetrain speed
    boolean upFlag = false;                 // Variables used in determining drivetrain speed
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    int count = 0;                          // Variables used in field centric driver code
    double angleTest[] = new double[10];
    double sum;
    double correct;
    double STARTING_HEADING = 90;
    float currentHeading = 0;               // Reading from IMU
    boolean isBlue = true;                  // Determines carousel direction
    public MaximusPrimeBase(OpMode theOpMode){
        opMode = theOpMode;
        Configuration();
    }
    public void DeliverBlockTele() {
        // Raise the lift
        if (!manualLift && opMode.gamepad2.dpad_down && liftAvailability.equals("Low")) {
            liftM.setPower(.75);
            liftAvailability = "Raising";
        }
        // Stop the lift at the correct level
        if (!manualLift && liftM.getCurrentPosition() > liftHeight && liftAvailability.equals("Raising")) {
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
        if (!manualLift && deliverBlockTimer.seconds() > 2 && liftAvailability.equals("Delivering")) {
            deliveryS.setPosition(0);
            liftAvailability = ("Lowering");
            liftM.setPower(-.75);
        }
        // Stop the lift at the correct level
        if (!manualLift && liftM.getCurrentPosition() < 0 && liftAvailability.equals("Lowering")) {
            liftM.setPower(0);
            liftAvailability = "Low";
        }
    }
    // All driver controls except for drivetrain controls
    public void DriverControls() {
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
    // All operator controls
    public void OperatorControls() {
        // Toggle auto lift
        if (opMode.gamepad2.a) {
            manualLift = true;
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
    // Telemetry to be displayed in teleop
    public void Telemetry() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = (angles.firstAngle);
        opMode.telemetry.addData("Current heading: ", currentHeading);
        opMode.telemetry.addData("Start heading", STARTING_HEADING);
        opMode.telemetry.addData("Lift: ", liftM.getCurrentPosition());
        opMode.telemetry.addData("", liftAvailability);
        opMode.telemetry.addData("Is lift manual? ", manualLift);
        opMode.telemetry.update();
    }
    // Field-centric driver code
    public void UpdateDriveTrain() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double speed = Math.hypot(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4);
        angle += angles.firstAngle - (Math.PI)/2 + STARTING_HEADING;
        double turnPower = opMode.gamepad1.right_stick_x;
        if(turnPower == 0){
            if (count < 10) {
                angleTest[count] = angle;
                angleTest[count] = angle;
                count++;
            }
            else if (count >= 10){
                sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0] + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8] + angleTest[9])/10;
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
        lfDrivetrainM.setPower((((speed * -(Math.sin(angle)) + turnPower)))*drvTrnSpd);
        lbDrivetrainM.setPower((((speed * -(Math.cos(angle)) + turnPower)))*drvTrnSpd);
        rfDrivetrainM.setPower((((speed * (Math.cos(angle))) + turnPower))*drvTrnSpd);
        rbDrivetrainM.setPower((((speed * (Math.sin(angle))) + turnPower))*drvTrnSpd);
    }
    public void ResetHeading(){ // Resets the imu heading by adding/subtracting from itself
        if (opMode.gamepad1.b){ // In case something goes wrong, driver can reposition the robot and reset the heading during teleop
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (angles.firstAngle > 0){
                angles.firstAngle -= 2 * angles.firstAngle;
            }
            else {
                angles.firstAngle += 2 * angles.firstAngle;
            }
            STARTING_HEADING = angles.thirdAngle;
        }
    }
    void IMUTurn(float targetAngle, String leftOrRight, int maxSpeedAngle, float minSpeed, float timeOutA) {
        // Resetting our timer
        imuTimer.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& imuTimer.seconds() < timeOutA) {
            // Updating our current heading while turning
            // This records the z axis angle in the variable 'currentHeading'
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
                lfDrivetrainM.setPower(-power);
                lbDrivetrainM.setPower(-power);
                rfDrivetrainM.setPower(power);
                rbDrivetrainM.setPower(power);
            } else {
                // Actually applying the power
                lfDrivetrainM.setPower(power);
                lbDrivetrainM.setPower(power);
                rfDrivetrainM.setPower(-power);
                rbDrivetrainM.setPower(-power);
            }
        }
        // Stopping the motors once we completed our turn
        lfDrivetrainM.setPower(0);
        lbDrivetrainM.setPower(0);
        rfDrivetrainM.setPower(0);
        rbDrivetrainM.setPower(0);
    }
    // Returns true if we are in initialization
    boolean IsInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }
    // Pauses code for a set length
    public void Sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }
    public void TeleopAllianceDetermination() {
        if (opMode.gamepad1.x) {
            isBlue = false;
        } else if (opMode.gamepad1.b) {
            isBlue = true;
        }
        opMode.telemetry.addData("Press X for red and B for blue", "");
        opMode.telemetry.addData("Are we blue? ", isBlue);
        opMode.telemetry.update();
    }
    public void Configuration() {
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