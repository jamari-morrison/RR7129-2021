package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Maximus Prime Tele", group = "Iterative Opmode")
public class MaximusPrimeTele extends OpMode {
    MaximusPrimeBase base;
    @Override
    public void init() {
        base = new MaximusPrimeBase(this);
    }

    public void init_loop() {
        base.TeleopAllianceDetermination();
    }

    @Override
    public void loop() {
        base.DriverControls();
        base.OperatorControls();
        base.UpdateDriveTrain();
        base.ResetHeading();
        base.DeliverBlockTele();
        base.Telemetry();
    }

    @Override
    public void stop() {
    }
}