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
        // 1. Map the Limelight hardware from the robot configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 2. Set the tracking pipeline to 1 as requested
        telemetry.addLine("Initializing Limelight to Pipeline 0 (Red Alliance)...");
        limelight.pipelineSwitch(0);

        // 3. Start processing frames
        limelight.start();

        telemetry.addData("Status", "Pipeline 0 Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 4. Retrieve the latest vision processing results
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get global robot pose on the field if localization is configured
                Pose3D botpose = result.getBotpose();

                // Get the individual tracked tags inside the cluster
                List<FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    for (FiducialResult fiducial : fiducials) {
                        // 5. Query the 3D target coordinates relative to the robot base
                        Pose3D targetPose = fiducial.getRobotPoseTargetSpace();

                        if (targetPose != null) {
                            // 6. Extract the cluster orientation yaw angle in degrees
                            double targetYaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                            int tagId = fiducial.getFiducialId();

                            // 7. Disregard the cluster tag if it falls outside -90.0 to 90.0 degrees
                            if (targetYaw < -90.0 || targetYaw > 90.0) {
                                telemetry.addData("Rejected Tag ID " + tagId, "Outside Target Window (Yaw: %.1f)", targetYaw);
                                continue; // Skips to the next tag in the frame
                            }

                            // 8. TARGET PROCESSED: Safe execution parameters go here
                            telemetry.addLine(String.format("\n== TARGET LOCKED: ID %d == ", tagId));
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
                    telemetry.addLine("No active cluster elements tracked inside Pipeline 1.");
                }
            } else {
                telemetry.addLine("Searching for Red Alliance cluster tags...");
            }

            telemetry.update();
            sleep(20); // Small thread sleep to keep CPU usage low
        }

        // 9. Stop vision camera processing when the OpMode terminates
        limelight.stop();
    }
}