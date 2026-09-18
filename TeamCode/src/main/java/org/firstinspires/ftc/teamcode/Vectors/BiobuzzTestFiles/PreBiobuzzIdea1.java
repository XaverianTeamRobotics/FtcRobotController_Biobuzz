package org.firstinspires.ftc.teamcode.Vectors.BiobuzzTestFiles;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "PreBiobuzzIdea1", group = "Biobuzz")
public class PreBiobuzzIdea1 extends OpMode {
    private Follower follower;
    private double double1;
    DcMotor intake, roller, motor6, motor7;
    Servo rollerservo;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        //change starting pose
        follower.setStartingPose(new Pose(0,0,Math.toRadians(90)));

        intake = hardwareMap.get(DcMotor.class, "motor4");
        roller = hardwareMap.get(DcMotor.class, "motor5");
        motor6 = hardwareMap.get(DcMotor.class, "motor6");
        motor7 = hardwareMap.get(DcMotor.class, "motor7");

        rollerservo = hardwareMap.get(Servo.class, "servo0");

        //motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start(){
        follower.startTeleOpDrive();
        rollerservo.setPosition(0.5);
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
            intake.setPower(0.7);
        }
        if (gamepad1.rightBumperWasReleased()){
            intake.setPower(0.0);
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
    // ANGLING
        if (gamepad2.left_stick_x != 0) {
            rollerservo.setPosition((gamepad2.left_stick_x + 1) / 2);
        }
    //REGULAR
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