package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Sens", group = "Sensor")
public class SensorColor extends LinearOpMode {
  NormalizedColorSensor colorSensor;
  View relativeLayout;
  @Override public void runOpMode() {
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


    float gain = 2;
    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }

    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {
      telemetry.addData("Distance (in)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH));
      telemetry.update();
    }
  }
}
