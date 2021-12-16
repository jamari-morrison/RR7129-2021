package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="Red", group="Iterative Opmode")
public class RedMiniBotTele extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    RedMiniBotBase base;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {base = new RedMiniBotBase(this);}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {base.movingDaPushBot();}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { base.stop();}
}

