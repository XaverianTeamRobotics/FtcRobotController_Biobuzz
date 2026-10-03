package org.firstinspires.ftc.teamcode.Vectors.Auto.Full;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name = "Blue Alliance Auto 5", group = "Autonomous")
public class BlueAllianceAuto5 extends LinearOpMode {

    // Drivetrain
    private DcMotor FL, BL, FR, BR;

    // Odometry Encoders
    // ODOMETRY CONFIGURATION: Change encoder motor assignments and ticks-per-inch constants below as needed.
    private DcMotor parallelEncoderLeft;
    private DcMotor parallelEncoderRight;
    private DcMotor perpendicularEncoder;

    private static final double ODO_TICKS_PER_INCH = 1892.37;
    private double currentOdoXInches = 0.0;
    private double currentOdoYInches = 0.0;
    private double currentOdoHeadingRad = 0.0;

    private int prevLeftTicks = 0;
    private int prevRightTicks = 0;
    private int prevPerpTicks = 0;

    // Subsystems
    private Limelight3A limelight;
    private CRServo turretServo;
    private DcMotor flywheel;
    private Servo colorServo;

    private static final double TURRET_GEAR_RATIO = 5.0;
    private static final double BASE_TURRET_GAIN  = 0.015;
    private static final double TURRET_P_GAIN     = BASE_TURRET_GAIN * TURRET_GEAR_RATIO;
    private static final double AIM_Y_OFFSET      = 4.0;

    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "motor0");
        BL = hardwareMap.get(DcMotor.class, "motor1");
        FR = hardwareMap.get(DcMotor.class, "motor2");
        BR = hardwareMap.get(DcMotor.class, "motor3");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        parallelEncoderLeft  = hardwareMap.get(DcMotor.class, "motor0");
        parallelEncoderRight = hardwareMap.get(DcMotor.class, "motor2");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "motor1");

        parallelEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        parallelEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight   = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "servo0");
        flywheel    = hardwareMap.get(DcMotor.class, "motor4");
        colorServo  = hardwareMap.get(Servo.class, "colorServo");

        colorServo.setPosition(0.5); // Rest position
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData("Status", "Blue Auto Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Drive forward 36 inches using Odometry feedback
            driveToOdometryY(36.0, 0.4);

            // STEP 2: Auto-Aim with Limelight and Launch for 4.0 seconds
            stateTimer.reset();
            while (opModeIsActive() && stateTimer.seconds() < 4.0) {
                updateOdometryPosition();
                autoAimAndLaunch();
            }

            // Stop motors
            setDrivePower(0, 0, 0, 0);
            flywheel.setPower(0);
            turretServo.setPower(0);
        }

        limelight.stop();
    }

    /**
     * Drives forward/backward to a specific Y target coordinate using Odometry feedback.
     */
    private void driveToOdometryY(double targetYInches, double power) {
        while (opModeIsActive() && Math.abs(currentOdoYInches - targetYInches) > 1.0) {
            updateOdometryPosition();

            double error = targetYInches - currentOdoYInches;
            double drivePower = Math.signum(error) * Math.min(Math.abs(power), Math.abs(error) * 0.05);

            setDrivePower(drivePower, drivePower, drivePower, drivePower);

            telemetry.addData("Target Y", targetYInches);
            telemetry.addData("Current Y", currentOdoYInches);
            telemetry.update();
        }
        setDrivePower(0, 0, 0, 0);
    }

    private void autoAimAndLaunch() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (FiducialResult fiducial : fiducials) {
                    Pose3D targetPose = fiducial.getRobotPoseTargetSpace();

                    if (targetPose != null) {
                        double targetYaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                        double relY = targetPose.getPosition().y + AIM_Y_OFFSET;

                        turretServo.setPower(Math.max(-1.0, Math.min(1.0, targetYaw * TURRET_P_GAIN)));
                        flywheel.setPower(calculateFlywheelPowerContinuous(relY));
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

    private void setDrivePower(double fl, double bl, double fr, double br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }
}