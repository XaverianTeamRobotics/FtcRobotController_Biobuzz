package org.firstinspires.ftc.teamcode.Vectors.Auto.Work;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name = "Blue Alliance Auto 3")
public class BlueAllianceAuto3 extends LinearOpMode {

    // Hardware Declarations
    private DcMotor FL, BL, FR, BR;
    private Limelight3A limelight;
    private CRServo turretServo;  // servo0
    private DcMotor flywheel;     // motor4
    private DcMotor feeder;       // motor5

    private static final double TURRET_P_GAIN = 0.02;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors
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

        // Initialize Shooter & Turret Hardware
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "servo0");
        flywheel    = hardwareMap.get(DcMotor.class, "motor4");
        feeder      = hardwareMap.get(DcMotor.class, "motor5");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Switch Limelight to Pipeline 1 (Configured for Blue Alliance Tag IDs)
        telemetry.addLine("Initializing Limelight to Pipeline 1 (Blue Alliance)...");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData("Status", "Pipeline 1 Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Drive from Start (88, 8) to Shooting Spot (88, 36)
            driveTimed(0.4, 0.0, 0.0, 1500);

            // STEP 2: Spin Up Flywheel & Aim Turret with Limelight AprilTags
            flywheel.setPower(0.85);
            long aimStartTime = System.currentTimeMillis();
            while (opModeIsActive() && (System.currentTimeMillis() - aimStartTime < 1000)) {
                trackTurretTarget("Blue");
            }

            // STEP 3: Shoot 3 Balls while maintaining AprilTag lock
            for (int i = 0; i < 3; i++) {
                if (!opModeIsActive()) break;

                feeder.setPower(1.0);
                sleep(300);
                feeder.setPower(0.0);

                sleep(400); // Flywheel speed recovery delay
            }

            flywheel.setPower(0.0);
            turretServo.setPower(0.0);

            // STEP 4: Drive from Shooting Spot (88, 36) to Blue Loading Zone (136, 115)
            driveTimed(0.0, 0.5, 0.0, 2200);  // Strafe right
            driveTimed(0.5, 0.0, 0.0, 3500);  // Drive forward to Loading Zone
        }

        limelight.stop();
    }

    /**
     * Drives the robot using 4-wheel mecanum controls while actively aiming the turret using AprilTags.
     */
    private void driveTimed(double driveY, double strafeX, double turnR, long durationMs) {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < durationMs)) {
            double denominator = Math.max(Math.abs(driveY) + Math.abs(strafeX) + Math.abs(turnR), 1.0);

            FL.setPower((driveY + strafeX + turnR) / denominator);
            BL.setPower((driveY - strafeX + turnR) / denominator);
            FR.setPower((driveY - strafeX - turnR) / denominator);
            BR.setPower((driveY + strafeX - turnR) / denominator);

            trackTurretTarget("Blue"); // Continuously track tags during movement
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    /**
     * Processes Limelight AprilTag results to point turret launcher toward the targeted cluster.
     */
    private void trackTurretTarget(String allianceName) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            List<FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (FiducialResult fiducial : fiducials) {
                    Pose3D targetPose = fiducial.getRobotPoseTargetSpace();

                    if (targetPose != null) {
                        double targetYaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                        int tagId = fiducial.getFiducialId();

                        // Ignore targets outside the target window (-90.0 to 90.0 degrees)
                        if (targetYaw < -90.0 || targetYaw > 90.0) {
                            telemetry.addData("Rejected Tag ID " + tagId, "Outside Window (Yaw: %.1f)", targetYaw);
                            continue;
                        }

                        // Calculate proportional servo power based on target yaw offset
                        double servoPower = targetYaw * TURRET_P_GAIN;
                        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
                        turretServo.setPower(servoPower);

                        telemetry.addLine(String.format("== %s TARGET LOCKED: ID %d ==", allianceName.toUpperCase(), tagId));
                        telemetry.addData("Target Yaw (Rotation)", "%.2f deg", targetYaw);
                        telemetry.addData("Relative X (Left/Right)", "%.2f in", targetPose.getPosition().x);
                        telemetry.addData("Relative Y (Forward/Back)", "%.2f in", targetPose.getPosition().y);

                        if (botpose != null) {
                            telemetry.addData("Field Localization X", "%.2f in", botpose.getPosition().x);
                            telemetry.addData("Field Localization Y", "%.2f in", botpose.getPosition().y);
                        }
                        telemetry.update();
                        return; // Lock onto first valid tag in cluster
                    }
                }
            } else {
                telemetry.addLine("No active cluster elements tracked inside Pipeline 1.");
            }
        } else {
            telemetry.addLine("Searching for Blue Alliance cluster tags...");
        }

        turretServo.setPower(0.0); // Stop servo if no valid target found
        telemetry.update();
    }
}