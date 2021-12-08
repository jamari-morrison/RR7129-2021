package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp
public class TankOdometry extends LinearOpMode {
    DcMotor LFDriveMotor, LBDriveMotor, RFDriveMotor, RBDriveMotor;
    BNO055IMU imu;
    Orientation angles;
    int maxSpeedAngle = 120;
    float minSpeed = 0.2f;
    ElapsedTime imuTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        LFDriveMotor = hardwareMap.dcMotor.get("LFDriveMotor");
        LBDriveMotor = hardwareMap.dcMotor.get("LBDriveMotor");
        RFDriveMotor = hardwareMap.dcMotor.get("RFDriveMotor");
        RBDriveMotor = hardwareMap.dcMotor.get("RBDriveMotor");
        LFDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LBDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (opModeIsActive()) {
            TankOdo(-2, 4, "left");
        }
    }

    public static void TankOdo(double XCoordinate, double YCoordinate, String leftOrRight) {
        double sectorAngle = 0;
        if (XCoordinate > 0 && YCoordinate > 0) {
            sectorAngle = -90;
        }
        if (YCoordinate > 0 && XCoordinate < 0) {
            sectorAngle = 90;
        } else if (YCoordinate < 0 && XCoordinate < 0) {
            sectorAngle = 90;
        } else if (YCoordinate < 0 && XCoordinate > 0) {
            sectorAngle = 270;
        }
        double distanceToMove = Math.hypot(XCoordinate, YCoordinate);
        double closeToAngleToTurnRad = ((Math.atan(XCoordinate / YCoordinate)));
        double angleToTurnInDeg = (Math.toDegrees(closeToAngleToTurnRad)) + sectorAngle;
        if (angleToTurnInDeg > 180) {
            angleToTurnInDeg -= 360;
        }
        System.out.println(angleToTurnInDeg);
    }
}