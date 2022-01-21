package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "Bob teleop", group = "Iterative Opmode")
public class BobTeleop extends OpMode {
    BobBase base;
    @Override
    public void init() {
        // Device configuration
        base = new BobBase(this);

    }
    @Override
    public void loop() {
        // Running the functions that are in the base class
        base.OperatorControls();
        base.DriverControls();
        base.Telemetry();
    }
    @Override
    public void stop() {
    }
}

