package org.firstinspires.ftc.teamcode.Vectors.TeleOp.Full;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorAndLauncherServo", group = "Sensors")
public class ColorAndLauncherServo extends LinearOpMode {

    // Servo Position Constants
    private static final double SERVO_YELLOW = 1.0;
    private static final double SERVO_RED_BLUE = 0.0;
    private static final double SERVO_RESTING = 0.5;

    // Timing Constants (in seconds)
    private static final double VERIFICATION_TIME_SEC = 0.5;
    private static final double SERVO_HOLD_TIME_SEC = 1.0;

    // Detect color states
    public enum DetectedColor {
        YELLOW,
        RED,
        BLUE,
        NONE
    }

    // Timers
    private final ElapsedTime verificationTimer = new ElapsedTime();
    private final ElapsedTime servoActionTimer = new ElapsedTime();

    // State Tracking
    private DetectedColor pendingColor = DetectedColor.NONE;
    private DetectedColor verifiedColor = DetectedColor.NONE;
    private boolean isHoldingServo = false;

    @Override
    public void runOpMode() {
        // Initialize color/distance sensor from hardware map
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor0");
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor0");

        // Initialize servo from hardware map
        Servo colorServo = hardwareMap.get(Servo.class, "colorServo");

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

            // 1. Get raw color reading from sensor
            DetectedColor rawColor = getDetectedColor(red, green, blue, distanceCm);

            // 2. Handle Color Verification (Must read consistently for 0.5s)
            if (rawColor != DetectedColor.NONE) {
                if (rawColor != pendingColor) {
                    // New color spotted, restart verification timer
                    pendingColor = rawColor;
                    verificationTimer.reset();
                } else if (verificationTimer.seconds() >= VERIFICATION_TIME_SEC && !isHoldingServo) {
                    // Color verified for 0.5s and no active servo action is running
                    verifiedColor = pendingColor;
                    isHoldingServo = true;
                    servoActionTimer.reset(); // Start 1.0s servo hold timer
                }
            } else {
                // Sensor sees nothing/out of range: reset verification
                pendingColor = DetectedColor.NONE;
            }

            // 3. Control Servo Output
            double targetServoPosition = SERVO_RESTING;

            if (isHoldingServo) {
                if (servoActionTimer.seconds() < SERVO_HOLD_TIME_SEC) {
                    // Move servo based on verified color
                    if (verifiedColor == DetectedColor.YELLOW) {
                        targetServoPosition = SERVO_YELLOW;
                    } else if (verifiedColor == DetectedColor.RED || verifiedColor == DetectedColor.BLUE) {
                        targetServoPosition = SERVO_RED_BLUE;
                    }
                } else {
                    // 1.0-second timer complete: return to resting state
                    isHoldingServo = false;
                    verifiedColor = DetectedColor.NONE;
                    pendingColor = DetectedColor.NONE;
                }
            }

            colorServo.setPosition(targetServoPosition);

            // Telemetry Output
            telemetry.addData("Raw Color", rawColor);
            telemetry.addData("Pending Color", pendingColor);
            telemetry.addData("Verified Color", verifiedColor);
            telemetry.addData("Holding Servo", isHoldingServo);
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