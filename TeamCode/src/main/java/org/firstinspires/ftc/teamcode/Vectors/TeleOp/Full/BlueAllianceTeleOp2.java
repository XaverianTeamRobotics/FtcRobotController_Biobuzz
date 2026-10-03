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

@TeleOp(name = "Blue Alliance TeleOp 2", group = "TeleOp")
public class BlueAllianceTeleOp2 extends LinearOpMode {

    // Drive Motors
    private DcMotor FL, BL, FR, BR;

    // Odometry Encoders (3-Deadwheel Pod Setup)
    // ODOMETRY CONFIGURATION: Change encoder motor assignments and ticks-per-inch constants below as needed.
    private DcMotor parallelEncoderLeft;   // Left deadwheel encoder
    private DcMotor parallelEncoderRight;  // Right deadwheel encoder
    private DcMotor perpendicularEncoder;  // Perpendicular (strafe) deadwheel encoder

    // Odometry Tracking Variables
    private static final double ODO_TICKS_PER_INCH = 1892.37; // ADJUST THIS VALUE to match your odometry wheel specs
    private double currentOdoXInches = 0.0;
    private double currentOdoYInches = 0.0;
    private double currentOdoHeadingRad = 0.0;

    private int prevLeftTicks = 0;
    private int prevRightTicks = 0;
    private int prevPerpTicks = 0;

    // Subsystem Hardware
    private DcMotor intakeMotor;
    private Limelight3A limelight;
    private CRServo turretServo;  // servo0
    private DcMotor flywheel;     // motor4
    private Servo colorServo;     // Launcher sizing gate
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // Aiming Constants
    private static final double TURRET_GEAR_RATIO = 5.0;
    private static final double BASE_TURRET_GAIN  = 0.015;
    private static final double TURRET_P_GAIN     = BASE_TURRET_GAIN * TURRET_GEAR_RATIO;
    private static final double AIM_Y_OFFSET      = 4.0; // Hive offset adjustment

    // Launcher Servo Gate Sizing Positions
    private static final double LAUNCHER_SIZE_LARGE = 1.0;
    private static final double LAUNCHER_SIZE_SMALL = 0.0;
    private static final double LAUNCHER_SIZE_REST  = 0.5;

    private static final double VERIFICATION_TIME_SEC = 0.5;
    private static final double SERVO_HOLD_TIME_SEC = 1.0;

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

        // ODOMETRY INITIALIZATION: Map encoder cables plugged into motor ports
        parallelEncoderLeft  = hardwareMap.get(DcMotor.class, "motor0"); // Example port mapping
        parallelEncoderRight = hardwareMap.get(DcMotor.class, "motor2"); // Example port mapping
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "motor1"); // Example port mapping

        // Reset Encoder Counts
        parallelEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        parallelEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Subsystems
        intakeMotor    = hardwareMap.get(DcMotor.class, "intakeMotor");
        limelight      = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo    = hardwareMap.get(CRServo.class, "servo0");
        flywheel       = hardwareMap.get(DcMotor.class, "motor4");
        colorServo     = hardwareMap.get(Servo.class, "colorServo");
        colorSensor    = hardwareMap.get(ColorSensor.class, "colorSensor0");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor0");

        colorServo.setPosition(LAUNCHER_SIZE_REST);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData("Blue TeleOp", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Odometry Position continuously
            updateOdometryPosition();

            // Drivetrain Controls
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
            FL.setPower((drive + strafe + turn) / denom);
            BL.setPower((drive - strafe + turn) / denom);
            FR.setPower((drive - strafe - turn) / denom);
            BR.setPower((drive + strafe - turn) / denom);

            // Intake Logic
            if (gamepad2.right_trigger > 0.1) {
                intakeMotor.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1) {
                intakeMotor.setPower(-gamepad2.left_trigger);
            } else {
                intakeMotor.setPower(0.0);
            }

            // Continuous Dynamic Aiming & Flywheel Calculation
            if (gamepad2.a) {
                updateTargetingAndFlywheelContinuous();
            } else {
                turretServo.setPower(gamepad2.right_stick_x);
                if (gamepad2.b) {
                    flywheel.setPower(0.85);
                } else {
                    flywheel.setPower(0.0);
                }
            }

            // Color Verification and Gate Sizing
            processColorAndAdjustLauncher();

            // Telemetry Outputs
            telemetry.addData("Odometry X (in)", "%.2f", currentOdoXInches);
            telemetry.addData("Odometry Y (in)", "%.2f", currentOdoYInches);
            telemetry.addData("Odometry Heading (deg)", "%.2f", Math.toDegrees(currentOdoHeadingRad));
            telemetry.addData("Flywheel Power", "%.2f %%", flywheel.getPower() * 100.0);
            telemetry.update();
        }

        limelight.stop();
    }

    /**
     * ODOMETRY CALCULATION: Calculates displacement from encoder delta ticks every loop iteration.
     */
    private void updateOdometryPosition() {
        int currentLeft  = parallelEncoderLeft.getCurrentPosition();
        int currentRight = parallelEncoderRight.getCurrentPosition();
        int currentPerp  = perpendicularEncoder.getCurrentPosition();

        double deltaLeftTicks  = currentLeft - prevLeftTicks;
        double deltaRightTicks = currentRight - prevRightTicks;
        double deltaPerpTicks  = currentPerp - prevPerpTicks;

        prevLeftTicks  = currentLeft;
        prevRightTicks = currentRight;
        prevPerpTicks  = currentPerp;

        double deltaYInches = ((deltaLeftTicks + deltaRightTicks) / 2.0) / ODO_TICKS_PER_INCH;
        double deltaXInches = deltaPerpTicks / ODO_TICKS_PER_INCH;

        currentOdoYInches += deltaYInches * Math.cos(currentOdoHeadingRad) - deltaXInches * Math.sin(currentOdoHeadingRad);
        currentOdoXInches += deltaYInches * Math.sin(currentOdoHeadingRad) + deltaXInches * Math.cos(currentOdoHeadingRad);
    }

    private void updateTargetingAndFlywheelContinuous() {
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

                        double targetFlywheelPower = calculateFlywheelPowerContinuous(relY);
                        flywheel.setPower(targetFlywheelPower);
                        return;
                    }
                }
            }
        }
        turretServo.setPower(0.0);
    }

    private double calculateFlywheelPowerContinuous(double distanceYInches) {
        double minDistance = 20.0;
        double maxDistance = 120.0;
        double minPower = 0.50;
        double maxPower = 0.98;

        double clampedY = Math.max(minDistance, Math.min(maxDistance, distanceYInches));
        return minPower + ((clampedY - minDistance) / (maxDistance - minDistance)) * (maxPower - minPower);
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
                } else if (verifiedColor == DetectedColor.RED) { // RED Alliance check
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

    private DetectedColor getDetectedColor(int r, int g, int b, double distanceCm) {
        if (distanceCm > 5.0) return DetectedColor.NONE;
        if (r > 1.2 * b && g > 1.2 * b && r > 50 && g > 50) return DetectedColor.YELLOW;
        if (r > g && r > b) return DetectedColor.RED;
        if (b > r && b > g) return DetectedColor.BLUE;
        return DetectedColor.NONE;
    }
}