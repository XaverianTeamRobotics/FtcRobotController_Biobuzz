package org.firstinspires.ftc.teamcode.Vectors.Auto.Work;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@Autonomous(name = "Red Alliance Auto 1")
public class RedAllianceAuto1 extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Map the Limelight hardware from the robot configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch the Limelight to use Pipeline 0 (Configured for Red Tag IDs)
        telemetry.addLine("Initializing Limelight to Pipeline 0 (Red Alliance)...");
        limelight.pipelineSwitch(0);

        // Start processing camera streams
        limelight.start();

        telemetry.addData("Status", "Pipeline 0 Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Retrieve the latest vision processing results
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get global robot pose on the field
                Pose3D botpose = result.getBotpose();

                // Get the individual tracked tags inside the cluster
                List<FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    for (FiducialResult fiducial : fiducials) {
                        // Query the 3D target coordinates relative to the robot base
                        Pose3D targetPose = fiducial.getRobotPoseTargetSpace();

                        if (targetPose != null) {
                            // Extract orientation angles in degrees
                            double targetYaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                            double tagPitch = targetPose.getOrientation().getPitch(AngleUnit.DEGREES);
                            double tagRoll = targetPose.getOrientation().getRoll(AngleUnit.DEGREES);
                            int tagId = fiducial.getFiducialId();

                            // Disregard the cluster tag if pitch falls outside acceptable tilt
                            if (Math.abs(tagPitch) > 30.0) {
                                telemetry.addData("Rejected Tag ID " + tagId, "Wrong Seesaw Tilt (Pitch: %.1f)", tagPitch);
                                continue;
                            }

                            // Disregard if outside horizontal viewing window
                            if (targetYaw < -90.0 || targetYaw > 90.0) {
                                telemetry.addData("Rejected Tag ID " + tagId, "Backside/Outside Window (Yaw: %.1f)", targetYaw);
                                continue;
                            }

                            // Disregard upside-down tags on the other side of the tip
                            if (Math.abs(tagRoll) > 90.0) {
                                telemetry.addData("Rejected Tag ID " + tagId, "Upside-Down Tag (Roll: %.1f)", tagRoll);
                                continue;
                            }

                            // TARGET PROCESSED: Safe tracking data is verified here
                            telemetry.addLine(String.format("\n== RED TARGET LOCKED: ID %d == ", tagId));
                            telemetry.addData("Target Yaw (Rotation)", "%.2f deg", targetYaw);
                            telemetry.addData("Relative X (Left/Right)", "%.2f in", targetPose.getPosition().x);
                            telemetry.addData("Relative Y (Forward/Back)", "%.2f in", targetPose.getPosition().y);

                            if (botpose != null) {
                                telemetry.addData("Field Localization X", "%.2f in", botpose.getPosition().x);
                                telemetry.addData("Field Localization Y", "%.2f in", botpose.getPosition().y);
                            }
                        }
                    }
                } else {
                    telemetry.addLine("No active cluster elements tracked inside Pipeline 0.");
                }
            } else {
                telemetry.addLine("Searching for Red Alliance cluster tags...");
            }

            telemetry.update();
            sleep(20);
        }

        // Stop vision camera processing when the OpMode terminates
        limelight.stop();
    }
}