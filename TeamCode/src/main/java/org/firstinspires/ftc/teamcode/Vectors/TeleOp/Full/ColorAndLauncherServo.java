package org.firstinspires.ftc.teamcode.Vectors.TeleOp.Full;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorAndLauncherServo", group = "Sensors")
public class ColorAndLauncherServo extends LinearOpMode {

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Servo colorServo;

    // Servo Position Constants
    private static final double SERVO_YELLOW = 1.0;
    private static final double SERVO_RED_BLUE = 0.0;
    private static final double SERVO_RESTING = 0.5;

    // Detected color states
    public enum DetectedColor {
        YELLOW,
        RED,
        BLUE,
        NONE
    }

    @Override
    public void runOpMode() {
        // Initialize color/distance sensor from hardware map
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor0");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor0");

        // Initialize servo from hardware map
        colorServo = hardwareMap.get(Servo.class, "colorServo");

        // Set servo to initial resting position
        colorServo.setPosition(SERVO_RESTING);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read RGB channels
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Read proximity distance in cm
            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

            // Determine detected color
            DetectedColor currentColor = getDetectedColor(red, green, blue, distanceCm);

            // Actuate servo based on detected color
            double targetServoPosition = SERVO_RESTING;

            switch (currentColor) {
                case YELLOW:
                    targetServoPosition = SERVO_YELLOW; // Position 1.0
                    break;
                case RED:
                case BLUE:
                    targetServoPosition = SERVO_RED_BLUE; // Position 0.0
                    break;
                case NONE:
                default:
                    targetServoPosition = SERVO_RESTING; // Position 0.5
                    break;
            }

            colorServo.setPosition(targetServoPosition);

            // Telemetry output
            telemetry.addData("Detected Color", currentColor);
            telemetry.addData("Servo Position", "%.2f", targetServoPosition);
            telemetry.addData("Distance (cm)", "%.2f", distanceCm);
            telemetry.addData("RGB Values", "R: %d | G: %d | B: %d", red, green, blue);
            telemetry.update();
        }
    }
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