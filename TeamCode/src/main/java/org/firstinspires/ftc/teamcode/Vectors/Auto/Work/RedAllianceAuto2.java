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

@Autonomous(name = "Red Alliance Auto 2")
public class RedAllianceAuto2 extends LinearOpMode {

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

        // Pipeline 0 for Red Alliance AprilTags
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Red Alliance Auto 2 Ready.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Drive from Start (56, 8) to Shooting Position (56, 36) -> Drive forward 28 inches
            driveTimed(0.4, 0.0, 0.0, 1500); // Drives forward to shooting spot

            // STEP 2: Spin Up Flywheel & Aim Turret
            flywheel.setPower(0.85);
            long aimStartTime = System.currentTimeMillis();
            while (opModeIsActive() && (System.currentTimeMillis() - aimStartTime < 1000)) {
                trackTurretTarget();
            }

            // STEP 3: Shoot 3 Balls
            for (int i = 0; i < 3; i++) {
                if (!opModeIsActive()) break;

                feeder.setPower(1.0);
                sleep(300);
                feeder.setPower(0.0);

                sleep(400);
            }

            flywheel.setPower(0.0);
            turretServo.setPower(0.0);

            // STEP 4: Move from Shooting Spot (56, 36) to Loading Zone (8, 115)
            // Strafe left toward X = 8, then drive forward toward Y = 115
            driveTimed(0.0, -0.5, 0.0, 2200); // Strafe left
            driveTimed(0.5, 0.0, 0.0, 3500);  // Drive forward to Red Loading Zone
        }

        limelight.stop();
    }

    /**
     * Drives the robot using 4-wheel mecanum controls for a specified duration in milliseconds.
     */
    private void driveTimed(double driveY, double strafeX, double turnR, long durationMs) {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < durationMs)) {
            double denominator = Math.max(Math.abs(driveY) + Math.abs(strafeX) + Math.abs(turnR), 1.0);

            FL.setPower((driveY + strafeX + turnR) / denominator);
            BL.setPower((driveY - strafeX + turnR) / denominator);
            FR.setPower((driveY - strafeX - turnR) / denominator);
            BR.setPower((driveY + strafeX - turnR) / denominator);

            trackTurretTarget(); // Keep aiming turret during drive movements
        }

        // Stop all wheel movement
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    private void trackTurretTarget() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                for (FiducialResult fiducial : fiducials) {
                    Pose3D targetPose = fiducial.getRobotPoseTargetSpace();
                    if (targetPose != null) {
                        double targetYaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);

                        if (targetYaw >= -90.0 && targetYaw <= 90.0) {
                            double servoPower = targetYaw * TURRET_P_GAIN;
                            servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
                            turretServo.setPower(servoPower);
                            return;
                        }
                    }
                }
            }
        }
        turretServo.setPower(0.0);
    }
}