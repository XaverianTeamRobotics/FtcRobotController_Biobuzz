package org.firstinspires.ftc.teamcode.Vectors.Auto.Full;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name = "Red Alliance Auto 4", group = "Auto")
public class RedAllianceAuto4 extends LinearOpMode {

    // Drivetrain & Intakes
    private DcMotor FL, BL, FR, BR;
    private DcMotor intakeMotor; // Single motor driving both intakes

    // Mechanisms
    private Limelight3A limelight;
    private CRServo turretServo;  // servo0 (5:1 gear ratio)
    private DcMotor flywheel;     // motor4
    private Servo colorServo;     // Adjusts launcher sizing gate

    // Sensors
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // Servo Position Constants for Launcher Size Adjustment
    private static final double LAUNCHER_SIZE_LARGE = 1.0; // Servo position for large elements (Yellow)
    private static final double LAUNCHER_SIZE_SMALL = 0.0; // Servo position for small elements (Red/Blue)
    private static final double LAUNCHER_SIZE_REST  = 0.5; // Neutral/default gate size

    // Timing Constants
    private static final double VERIFICATION_TIME_SEC = 0.5;
    private static final double SERVO_HOLD_TIME_SEC = 1.0;

    // Limelight & Turret Control
    private static final double TURRET_GEAR_RATIO = 5.0;
    private static final double BASE_TURRET_GAIN  = 0.015;
    private static final double TURRET_P_GAIN     = BASE_TURRET_GAIN * TURRET_GEAR_RATIO;
    private static final double AIM_Y_OFFSET      = 4.0;

    public enum DetectedColor { YELLOW, RED, BLUE, NONE }

    private final ElapsedTime verificationTimer = new ElapsedTime();
    private final ElapsedTime servoActionTimer = new ElapsedTime();
    private DetectedColor pendingColor = DetectedColor.NONE;
    private DetectedColor verifiedColor = DetectedColor.NONE;
    private boolean isHoldingServo = false;

    @Override
    public void runOpMode() {
        // Drivetrain Initialization
        FL = hardwareMap.get(DcMotor.class, "motor0");
        BL = hardwareMap.get(DcMotor.class, "motor1");
        FR = hardwareMap.get(DcMotor.class, "motor2");
        BR = hardwareMap.get(DcMotor.class, "motor3");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hardware Mapping
        intakeMotor    = hardwareMap.get(DcMotor.class, "intakeMotor");
        limelight      = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo    = hardwareMap.get(CRServo.class, "servo0");
        flywheel       = hardwareMap.get(DcMotor.class, "motor4");
        colorServo     = hardwareMap.get(Servo.class, "colorServo");
        colorSensor    = hardwareMap.get(ColorSensor.class, "colorSensor0");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor0");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        colorServo.setPosition(LAUNCHER_SIZE_REST);

        // Switch Limelight to Pipeline 0 for Red Alliance
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Red Alliance Auto Initialized (Pipeline 0)");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Drive to initial firing area (Adjust duration when start pos is finalized)
            driveTimed(0.4, 0.0, 0.0, 1200);

            // STEP 2: Main Firing & Aiming Loop
            long autoStart = System.currentTimeMillis();
            while (opModeIsActive() && (System.currentTimeMillis() - autoStart < 15000)) {
                // Run Intake A forward to intake elements
                intakeMotor.setPower(1.0);

                // Dynamically adjust launcher size based on sensor reading
                processColorAndAdjustLauncher();

                // Track target and set flywheel velocity based on Limelight Y-distance
                trackTurretAndSetFlywheel();
            }

            // Stop Mechanisms
            intakeMotor.setPower(0.0);
            flywheel.setPower(0.0);
            turretServo.setPower(0.0);

            // STEP 3: Park in Red Zone (Adjust movements once start position is determined)
            driveTimed(0.0, -0.5, 0.0, 1800); // Strafe
            driveTimed(0.5, 0.0, 0.0, 2500);  // Drive forward
        }

        limelight.stop();
    }

    private void driveTimed(double driveY, double strafeX, double turnR, long durationMs) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < durationMs)) {
            double denom = Math.max(Math.abs(driveY) + Math.abs(strafeX) + Math.abs(turnR), 1.0);
            FL.setPower((driveY + strafeX + turnR) / denom);
            BL.setPower((driveY - strafeX + turnR) / denom);
            FR.setPower((driveY - strafeX - turnR) / denom);
            BR.setPower((driveY + strafeX - turnR) / denom);

            processColorAndAdjustLauncher();
            trackTurretAndSetFlywheel();
        }
        FL.setPower(0); BL.setPower(0); FR.setPower(0); BR.setPower(0);
    }

    private void trackTurretAndSetFlywheel() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                for (FiducialResult fiducial : fiducials) {
                    Pose3D targetPose = fiducial.getRobotPoseTargetSpace();
                    if (targetPose != null) {
                        double targetYaw   = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                        double targetPitch = targetPose.getOrientation().getPitch(AngleUnit.DEGREES);
                        double targetRoll  = targetPose.getOrientation().getRoll(AngleUnit.DEGREES);

                        // Reject inverted targets or invalid yaw angles
                        if (Math.abs(targetPitch) > 45.0 || Math.abs(targetRoll) > 135.0 || Math.abs(targetYaw) > 90.0) {
                            continue;
                        }

                        double relY = targetPose.getPosition().y + AIM_Y_OFFSET;

                        // Calculate turret control power (compensated for 5:1 gear ratio)
                        double servoPower = Math.max(-1.0, Math.min(1.0, targetYaw * TURRET_P_GAIN));
                        turretServo.setPower(servoPower);

                        // Calculate dynamic flywheel power for 65 deg launcher
                        flywheel.setPower(calculateFlywheelPower(relY));
                        return;
                    }
                }
            }
        }
        turretServo.setPower(0.0);
    }

    private void processColorAndAdjustLauncher() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

        DetectedColor rawColor = getDetectedColor(r, g, b, distanceCm);

        if (rawColor != DetectedColor.NONE) {
            if (rawColor != pendingColor) {
                pendingColor = rawColor;
                verificationTimer.reset();
            } else if (verificationTimer.seconds() >= VERIFICATION_TIME_SEC && !isHoldingServo) {
                verifiedColor = pendingColor;
                isHoldingServo = true;
                servoActionTimer.reset();
            }
        } else {
            pendingColor = DetectedColor.NONE;
        }

        double targetServoPosition = LAUNCHER_SIZE_REST;

        if (isHoldingServo) {
            if (servoActionTimer.seconds() < SERVO_HOLD_TIME_SEC) {
                // Adjust launcher aperture based on element size
                if (verifiedColor == DetectedColor.YELLOW) {
                    targetServoPosition = LAUNCHER_SIZE_LARGE; // Expand gate for large element
                } else if (verifiedColor == DetectedColor.RED || verifiedColor == DetectedColor.BLUE) {
                    targetServoPosition = LAUNCHER_SIZE_SMALL; // Contract gate for small element
                }
            } else {
                isHoldingServo = false;
                verifiedColor = DetectedColor.NONE;
                pendingColor = DetectedColor.NONE;
            }
        }

        colorServo.setPosition(targetServoPosition);
    }

    private double calculateFlywheelPower(double distanceYInches) {
        double minDistance = 24.0, maxDistance = 120.0;
        double minPower = 0.55, maxPower = 0.95;
        double clampedY = Math.max(minDistance, Math.min(maxDistance, distanceYInches));
        return minPower + ((clampedY - minDistance) / (maxDistance - minDistance)) * (maxPower - minPower);
    }

    private DetectedColor getDetectedColor(int r, int g, int b, double distanceCm) {
        if (distanceCm > 5.0) return DetectedColor.NONE;
        if (r > 1.2 * b && g > 1.2 * b && r > 50 && g > 50) return DetectedColor.YELLOW;
        if (r > g && r > b) return DetectedColor.RED;
        if (b > r && b > g) return DetectedColor.BLUE;
        return DetectedColor.NONE;
    }
}