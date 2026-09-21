package org.firstinspires.ftc.teamcode.Vectors.TeleOp.Work;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name="AprilTagDriveOpMode", group = "Biobuzz")
public class AprilTagDriveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Intake side is the front of the robot
        DcMotor FL = hardwareMap.get(DcMotor.class, "motor0");
        DcMotor BL = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor FR = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor BR = hardwareMap.get(DcMotor.class, "motor3");


        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);


        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double powerScale = 0.5;
        double rotScale   = 0.5;
        boolean bevelGears = false;

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 0"))
                .addProcessor(aprilTag)
                .build();


        final double DESIRED_DISTANCE = 12.0;
        final double SPEED_GAIN  = 0.02;
        final double STRAFE_GAIN = 0.015;
        final double TURN_GAIN   = 0.01;

        waitForStart();

        while (opModeIsActive()) {
            double y = 0; // forward
            double x = 0; // lateral
            double r = 0; // rotation


            boolean driverOverride = (Math.abs(gamepad1.left_stick_y)  > 0.05 ||
                    Math.abs(gamepad1.left_stick_x)  > 0.05 ||
                    Math.abs(gamepad1.right_stick_x) > 0.05);

            if (driverOverride) {
                y = -gamepad1.left_stick_y;  // push stick forward = positive y
                x =  gamepad1.left_stick_x;
                r =  gamepad1.right_stick_x;

            } else {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                boolean targetFound = false;
                AprilTagDetection matchedDetection = null;

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        targetFound = true;
                        matchedDetection = detection;

                        if (detection.id == 21) {
                            y =  (detection.ftcPose.range - DESIRED_DISTANCE) * SPEED_GAIN;
                            x =  detection.ftcPose.x * STRAFE_GAIN;
                            r =  (detection.ftcPose.bearing * TURN_GAIN) * rotScale;
                        } else if (detection.id == 20) {
                            y = 0; x = 0; r = 0;
                        }

                        break;
                    }
                }

                if (!targetFound) {
                    y = 0; x = 0; r = 0;
                }

                // Telemetry for debugging
                telemetry.addData("Status", targetFound ? "Tag Detected" : "Searching...");
                if (targetFound && matchedDetection != null) {
                    telemetry.addData("Target ID", matchedDetection.id);
                    telemetry.addData("Y (Drive)",   y);
                    telemetry.addData("X (Strafe)",  x);
                    telemetry.addData("R (Rotate)",  r);
                }
            }

            telemetry.addData("Mode", driverOverride ? "Manual" : "AprilTag");
            telemetry.update();

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1.0);
            int gearMod = bevelGears ? -1 : 1;

            FL.setPower( (y + x + r) / denominator * gearMod * powerScale);
            BL.setPower( (y - x + r) / denominator * gearMod * powerScale);
            FR.setPower( (y - x - r) / denominator * gearMod * powerScale);
            BR.setPower( (y + x - r) / denominator * gearMod * powerScale);
        }

        visionPortal.close();
    }
}