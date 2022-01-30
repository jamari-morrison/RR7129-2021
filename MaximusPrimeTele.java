package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
        base.AllianceDetermination();
    }

    @Override
    public void loop() {
        base.DeliverBlockTele();
        base.DriverControls();
        base.OperatorControls();
        base.UpdateDriveTrain();
        base.Telemetry();
    }

    @Override
    public void stop() {
    }
}