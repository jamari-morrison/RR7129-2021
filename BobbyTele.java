package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bobby Tele", group = "Iterative Opmode")
public class BobbyTele extends OpMode {
    BobbyBase base;
    @Override
    public void init() {
        base = new BobbyBase(this);
    }

    @Override
    public void loop() {
        base.DriverControls();
        base.OperatorControls();
    }

    @Override
    public void stop() {
    }
}