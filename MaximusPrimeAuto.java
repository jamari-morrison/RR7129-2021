package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Maximus Prime Auto", group="")
public class MaximusPrimeAuto extends LinearOpMode {
    MaximusPrimeBase base;

    @Override
    public void runOpMode() throws InterruptedException {
        base = new MaximusPrimeBase(this);
        while (base.IsInitialized()){
            base.AllianceDetermination();
        }
        waitForStart();
        base.Autonomous();
        while (opModeIsActive()) {
            base.Telemetry();
        }

    }
}