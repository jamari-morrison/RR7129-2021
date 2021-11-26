package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Disabled
@TeleOp(name = "Sensor: Color", group = "Sensor")
public class ColorTest extends LinearOpMode {
    // Set up the color sensor
    NormalizedColorSensor colorSensor;

    // Gain is how much to multiply the color readings by. Otherwise they'll be really small numbers
    float gain = 5;
    final float[] hsvValues = new float[3];
	//new comment!
	//
    // Variables for lines 61 - 75
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;

// ;lksad

    @Override public void runOpMode() {
        // Name the color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Turn on the color sensor's light if it's not already
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Run the "runSample" function
            runSample();
        }
    }

    protected void runSample() {


        // Update the gain value if either of the A or B gamepad buttons is being held
        if (gamepad1.a) {
            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
            gain += 0.005;
        } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
            gain -= 0.005;
        }

        // Tell the sensor our desired gain value
        colorSensor.setGain(gain);


        // This will make sure that it only toggles the light once per click

        // Check the status of the X button on the gamepad
        xButtonCurrentlyPressed = gamepad1.x;
        // If the button state is different than what it was, then act
        if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
            // If the button is (now) down, then toggle the light
            if (xButtonCurrentlyPressed) {
                if (colorSensor instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight)colorSensor;
                    light.enableLight(!light.isLightOn());
                }
            }
        }
        xButtonPreviouslyPressed = xButtonCurrentlyPressed;


        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Show the gain value via telemetry
        telemetry.addData("Gain", gain);

        // Display the RGB values
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.update();
    }
}
