package org.firstinspires.ftc.teamcode.Vectors.TeleOp.Work;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "PreBiobuzzCode", group = "Biobuzz")
public class PreBiobuzzTemplate extends OpMode {
    private Follower follower;
    private double double1;
    DcMotor motor4, motor5, motor6, motor7;
    Servo servo0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60.0,8.5,Math.toRadians(90)));


        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor5 = hardwareMap.get(DcMotor.class, "motor5");
        motor6 = hardwareMap.get(DcMotor.class, "motor6");
        motor7 = hardwareMap.get(DcMotor.class, "motor7");

        servo0 = hardwareMap.get(Servo.class, "servo0");


        //motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start(){
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y*(1.0-gamepad1.left_trigger), -gamepad1.left_stick_x*(1.0-gamepad1.left_trigger), -gamepad1.right_stick_x*(1.0-gamepad1.left_trigger), true);
        follower.update();
        telemetry.addData("x:",follower.getPose().getX());
        telemetry.addData("y:",follower.getPose().getY());
        telemetry.addData("angle", Math.toDegrees(follower.getPose().getHeading()));
        //telemetry.addData("Double1:", double1);
        telemetry.update();

// GAMEPAD 1 CONTROLS
        if (gamepad1.right_bumper){

        }
        if (gamepad1.left_bumper){

        }
        if (gamepad1.dpad_down){

        }
        if (gamepad1.dpad_left){

        }
        if (gamepad1.dpad_up){

        }
        if (gamepad1.dpad_right){

        }
        if (gamepad1.cross) {

        }
        if (gamepad1.circle){

        }
        if (gamepad1.triangle) {

        }
        if (gamepad1.square){

        }
// GAMEPAD 2 CONTROLS

        if (gamepad2.right_bumper){

        }
        if (gamepad2.left_bumper){

        }
        if (gamepad2.dpad_down){

        }
        if (gamepad2.dpad_left){

        }
        if (gamepad2.dpad_up){

        }
        if (gamepad2.dpad_right){

        }
        if (gamepad2.cross) {

        }
        if (gamepad2.circle){

        }
        if (gamepad2.triangle) {

        }
        if (gamepad2.square){

        }
    }
}