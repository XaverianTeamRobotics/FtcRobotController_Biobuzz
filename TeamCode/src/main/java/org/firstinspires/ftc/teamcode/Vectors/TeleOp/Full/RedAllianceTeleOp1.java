package org.firstinspires.ftc.teamcode.Vectors.TeleOp.Full;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "Red Alliance TeleOp 1", group = "TeleOp")
public class RedAllianceTeleOp1 extends LinearOpMode {

    // Drivetrain & Intakes
    private DcMotor FL, BL, FR, BR;
    private DcMotor intakeMotor; // Dual intake single motor

    // Mechanisms
    private Limelight3A limelight;
    private CRServo turretServo;  // servo0 (5:1 gear ratio)
    private DcMotor flywheel;     // motor4
    private Servo colorServo;     // Dynamic sizing servo for launcher

    // Sensors
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // Servo Position Constants for Launcher Gate Sizing
    private static final double LAUNCHER_SIZE_LARGE = 1.0; // Large size position
    private static final double LAUNCHER_SIZE_SMALL = 0.0; // Small size position
    private static final double LAUNCHER_SIZE_REST  = 0.5; // Rest/neutral position

    // Timing Constants
    private static final double VERIFICATION_TIME_SEC = 0.5;
    private static final double SERVO_HOLD_TIME_SEC = 1.0;

    // Control Gains
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
        // Drivetrain Motors
        FL = hardwareMap.get(DcMotor.class, "motor0");
        BL = hardwareMap.get(DcMotor.class, "motor1");
        FR = hardwareMap.get(DcMotor.class, "motor2");
        BR = hardwareMap.get(DcMotor.class, "motor3");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hardware Mapping
        intakeMotor    = hardwareMap.get(DcMotor.class, "intakeMotor");
        limelight      = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo    = hardwareMap.get(CRServo.class, "servo0");
        flywheel       = hardwareMap.get(DcMotor.class, "motor4");
        colorServo     = hardwareMap.get(Servo.class, "colorServo");
        colorSensor    = hardwareMap.get(ColorSensor.class, "colorSensor0");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor0");

        colorServo.setPosition(LAUNCHER_SIZE_REST);

        // Pipeline 0 configured
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Red TeleOp Ready (Pipeline 0)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // -------------------------------------------------------------
            // 1. GAMEPAD 1: Mecanum Drive Controls
            // -------------------------------------------------------------
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
            FL.setPower((drive + strafe + turn) / denom);
            BL.setPower((drive - strafe + turn) / denom);
            FR.setPower((drive - strafe - turn) / denom);
            BR.setPower((drive + strafe - turn) / denom);

            // -------------------------------------------------------------
            // 2. GAMEPAD 2: Dual Intake Control (Single Motor)
            // -------------------------------------------------------------
            if (gamepad2.right_trigger > 0.1) {
                intakeMotor.setPower(gamepad2.right_trigger);   // Intake A Forward
            } else if (gamepad2.left_trigger > 0.1) {
                intakeMotor.setPower(-gamepad2.left_trigger);  // Intake B Reverse
            } else {
                intakeMotor.setPower(0.0);
            }

            // -------------------------------------------------------------
            // 3. Limelight Targeting vs. Manual Override
            // -------------------------------------------------------------
            if (gamepad2.a) { // Press A to track target with Limelight
                trackTurretAndSetFlywheel();
            } else { // Manual Turret Control with Right Stick X
                turretServo.setPower(gamepad2.right_stick_x);
                if (gamepad2.b) {
                    flywheel.setPower(0.85); // Fixed flywheel speed override
                } else {
                    flywheel.setPower(0.0);
                }
            }

            // -------------------------------------------------------------
            // 4. Color Sensor Logic -> Launcher Sizing Servo
            // -------------------------------------------------------------
            processColorAndAdjustLauncher();

            // Telemetry
            telemetry.addData("Verified Color", verifiedColor);
            telemetry.addData("Launcher Gate Pos", "%.2f", colorServo.getPosition());
            telemetry.addData("Flywheel Power", "%.2f", flywheel.getPower());
            telemetry.update();
        }

        limelight.stop();
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

                        if (Math.abs(targetPitch) > 45.0 || Math.abs(targetRoll) > 135.0 || Math.abs(targetYaw) > 90.0) {
                            continue;
                        }

                        double relY = targetPose.getPosition().y + AIM_Y_OFFSET;
                        double servoPower = Math.max(-1.0, Math.min(1.0, targetYaw * TURRET_P_GAIN));
                        turretServo.setPower(servoPower);
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
                if (verifiedColor == DetectedColor.YELLOW) {
                    targetServoPosition = LAUNCHER_SIZE_LARGE;
                } else if (verifiedColor == DetectedColor.RED || verifiedColor == DetectedColor.BLUE) {
                    targetServoPosition = LAUNCHER_SIZE_SMALL;
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