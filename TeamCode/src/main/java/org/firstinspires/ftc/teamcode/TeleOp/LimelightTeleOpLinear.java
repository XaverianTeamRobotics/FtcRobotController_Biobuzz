package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight TeleOp (LinearOpMode)", group = "debug")
public class LimelightTeleOpLinear extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private DcMotor leftDrive;
    private DcMotor rightDrive;

        // Active TeleOp Match Loop



    @Override
    public void init() {
        // 1. Initialize Motors (Adjust motor names to match your Driver Station config)
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Initialize Limelight 3A Vision Sensor
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline index

        // 3. Initialize Control Hub IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry.addData("Status", "Initialized & Ready for TeleOp");
        telemetry.update();

        limelight.start();
    }

    @Override
    public void loop() {
        // A. Arcade Drive Controls (Left Stick Y = Forward/Back, Right Stick X = Turn)
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // B. Update Limelight with IMU Heading for MegaTag2 3D Pose
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYawDegrees = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(currentYawDegrees);

        // C. Query latest detection data from Limelight
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            telemetry.addData("Target Status", "AprilTag Locked");
            telemetry.addData("Robot X (m)", botPose.getPosition().x);
            telemetry.addData("Robot Y (m)", botPose.getPosition().y);
            telemetry.addData("Tx (deg)", llResult.getTx());
            telemetry.addData("Ty (deg)", llResult.getTy());
            telemetry.addData("Ta (%)", llResult.getTa());
        } else {
            telemetry.addData("Target Status", "No Target Detected");
        }

        telemetry.addData("Left Motor Power", leftPower);
        telemetry.addData("Right Motor Power", rightPower);
        telemetry.update();
    }
}