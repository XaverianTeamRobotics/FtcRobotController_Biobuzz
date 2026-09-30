package org.firstinspires.ftc.teamcode.Vectors.TeleOp.Full;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "REV Color Sensor V3 Test", group = "Sensors")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // Detected color states
    public enum DetectedColor {
        YELLOW,
        RED,
        BLUE,
        NONE
    }

    @Override
    public void runOpMode() {
        // Initialize sensor using the new configuration name "colorSensor0"
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor0");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor0");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read RGB channels
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Read proximity distance in centimeters
            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

            // Determine detected color
            DetectedColor currentColor = getDetectedColor(red, green, blue, distanceCm);

            // Output data to Telemetry
            telemetry.addData("Detected Color", currentColor);
            telemetry.addData("Distance (cm)", "%.2f", distanceCm);
            telemetry.addData("RGB Values", "R: %d | G: %d | B: %d", red, green, blue);
            telemetry.update();
        }
    }

    /**
     * Determines whether the sample is Yellow, Red, Blue, or None based on RGB and proximity threshold.
     */
    private DetectedColor getDetectedColor(int r, int g, int b, double distanceCm) {
        // Target object must be within 5 cm to trigger valid detection
        if (distanceCm > 5.0) {
            return DetectedColor.NONE;
        }

        // 1. Detect Yellow: High Red and Green combined with lower Blue
        if (r > 1.2 * b && g > 1.2 * b && r > 50 && g > 50) {
            return DetectedColor.YELLOW;
        }

        // 2. Detect Red: Red is dominant channel
        if (r > g && r > b) {
            return DetectedColor.RED;
        }

        // 3. Detect Blue: Blue is dominant channel
        if (b > r && b > g) {
            return DetectedColor.BLUE;
        }

        return DetectedColor.NONE;
    }
}