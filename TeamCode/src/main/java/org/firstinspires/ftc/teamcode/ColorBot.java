/*
ColorBot.java is a class extending LinearOpMode that reads input from the combined range and color
sensor. The methods isBlack() and isYellow() are intended to be utilities to be called by the
autonomous OpMode in order to determine whether a block in front of the sensor is Yellow or Black.

The purpose of the main "RunOpMode" method within ColorBot.java is to test and calibrate the two
utility functions, isBlack() and isYellow().

*/


package org.firstinspires.ftc.teamcode;

// Importing libraries
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp(name = "ColorBot", group = "Opmode")


public class ColorBot extends LinearOpMode {

    private ColorSensor sensorColorRangeAsREVColorRangeSensor;

    /* The four values below should be tested and calibrated in varying light conditions */
    static double yellowLowerBound = 15;
    static double yellowUpperBound = 30;
    static double yellowSaturationThreshold = 0.56;
    static double blackValueThreshold = 0.2;

    @Override
    public void runOpMode() {
        //Declare variables that will hold the values of the sensor's output in HSV
        int colorRGB;
        float hue; // Hue: what color (Angle around the color wheel)
        float sat; // Saturation: how much color (0 to 1)
        float val; // Value : how much light (0 to 1)

        //Hardware map for the color sensor
        sensorColorRangeAsREVColorRangeSensor = hardwareMap.colorSensor.get("sensorColorRangeAsREVColorRangeSensor");

        telemetry.update();
        waitForStart();

        //(if (we press play on the phone){

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Display distance info.
                //telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorColorRangeAsREVColorRangeSensor).getDistance(DistanceUnit.CM));
                // Display reflected light.ColorSensor
                //telemetry.addData("Light detected", ((OpticalDistanceSensor) sensorColorRangeAsREVColorRangeSensor).getLightDetected());


                // Get the RGB values from the sensor and store them in colorRGB

                colorRGB = Color.argb(sensorColorRangeAsREVColorRangeSensor.alpha(), sensorColorRangeAsREVColorRangeSensor.red(), sensorColorRangeAsREVColorRangeSensor.green(), sensorColorRangeAsREVColorRangeSensor.blue());
                // Converting from RGB to HSV
                // Get hue.
                hue = JavaUtil.colorToHue(colorRGB);
                // Get saturation.
                sat = JavaUtil.colorToSaturation(colorRGB);
                // Get value.
                val = JavaUtil.colorToValue(colorRGB);

                //Print the data to the phone so that we can check as we test the color sensor
                telemetry.addData("(H,S,V) =", "(" + hue + ", " + sat + ", " + val + ")");

                if (isBlack(sensorColorRangeAsREVColorRangeSensor)) {
                    telemetry.addData("Color", "Black");
                } else if (isYellow(sensorColorRangeAsREVColorRangeSensor)) {
                    telemetry.addData("Color", "Yellow");
                } else telemetry.addData("Color", "Error");
                telemetry.update();
            }
        }
    }

    /*
    The method isBlack() must be passed a ColorSensor as an argument; it then reads the RGB value
    from the hardware mapped ColorSensor. The RGB value is converted to the "V" Value from the
    HSV color scale. The method returns true if and only if the HSV Value from the ColorSensor is
    greater than the blackValueThreshold defined as a class variable.
     */
    public static boolean isBlack(ColorSensor sensorColorRangeAsREVColorRangeSensor) {

        int colorRGB;
        float val;


        // Get RGB color from sensor
        colorRGB = Color.argb(sensorColorRangeAsREVColorRangeSensor.alpha(), sensorColorRangeAsREVColorRangeSensor.red(), sensorColorRangeAsREVColorRangeSensor.green(), sensorColorRangeAsREVColorRangeSensor.blue());
        // Get Value and compare it to static variable blackValueThreshold
        val = JavaUtil.colorToValue(colorRGB);
        if (val < blackValueThreshold) {
            return true;
        } else return false;
    }

   /*
   The method isYellow() must be passed a ColorSensor as an argument; it then reads the RGB value
   from the hardware mapped ColorSensor. The RGB value is converted to Hue, Saturation, and Value.
   The method returns true if and only if the Hue, Saturation, and Value of the sensor's output are
   within the constraints set by the static class parameters blackValueThreshold, yellowLowerBound,
   yellowUpperBound, and yellowSaturationThreshold.
    */

    public static boolean isYellow(ColorSensor sensorColorRangeAsREVColorRangeSensor) {

        int colorRGB;
        float hue;
        float sat;
        float val;

        colorRGB = Color.argb(sensorColorRangeAsREVColorRangeSensor.alpha(), sensorColorRangeAsREVColorRangeSensor.red(), sensorColorRangeAsREVColorRangeSensor.green(), sensorColorRangeAsREVColorRangeSensor.blue());
        // Converting from RGB to HSV
        // Get hue.
        hue = JavaUtil.colorToHue(colorRGB);
        // Get saturation.
        sat = JavaUtil.colorToSaturation(colorRGB);
        // Get value.
        val = JavaUtil.colorToValue(colorRGB);

        if (val > blackValueThreshold) {
            if ((hue > yellowLowerBound) && (hue < yellowUpperBound)) {
                if (sat > yellowSaturationThreshold) {
                    return true;
                } else return false;
            } else return false;
        } else return false;

    }
}


