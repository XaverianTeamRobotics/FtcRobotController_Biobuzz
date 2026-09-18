package org.firstinspires.ftc.teamcode.Vectors.BiobuzzTestFiles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PreBiobuzzIdea1noPedro", group = "Biobuzz")
public class PreBiobuzzIdea1noPedro extends OpMode {
    private double double1;
    DcMotor left, right, intake;
    Servo rollerservo;

    @Override
    public void init() {

        left = hardwareMap.get(DcMotor.class, "motor1");
        right = hardwareMap.get(DcMotor.class,"motor0");
        intake = hardwareMap.get(DcMotor.class, "motor2");

        rollerservo = hardwareMap.get(Servo.class, "servo0");

        //motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start(){
        rollerservo.setPosition(0.5);
    }

    @Override
    public void loop() {
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);


// GAMEPAD 1 CONTROLS
        if (gamepad1.right_bumper){
            intake.setPower(-1.0);
        }
        if (gamepad1.left_bumper){
            intake.setPower(1.0);
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