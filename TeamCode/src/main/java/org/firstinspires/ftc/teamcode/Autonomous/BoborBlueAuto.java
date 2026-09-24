package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "BoborBlueAuto", group = "Autonomous", preselectTeleOp = "Bobor2b_alt")
public class BoborBlueAuto extends OpMode {
    private Follower follower;
    Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotor launcher, lifty, intake;
    private Servo back;
    private boolean waitingForScore = false;
    private int pathState;
    private final Pose scorePose = new Pose(144-85.0,85.0,Math.toRadians(135));
    private final Pose pickupPose = new Pose(144-127.6,35.2,180.0);
    private Path move;
    PathChain score1, reload, reload2, pickup, score2, finish;

    public void buildPaths() {
        move = new Path(new BezierLine(new Pose(144-84.0, 8.5, Math.toRadians(90)), (new Pose(144-84.0, 9.1))));
        move.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));

        score1 = follower.pathBuilder()
                .addPath(new BezierLine((new Pose(144-84.0, 9.1)), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), scorePose.getHeading())
                .addParametricCallback(0.98, () -> {
                    if (waitingForScore) return;
                    follower.pausePathFollowing();
                    lifty.setPower(-1.0);
                    actionTimer.resetTimer();
                    waitingForScore = true;
                })
                .build();
        reload = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(144-88.0,88.0, Math.toRadians(135))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(135))
                .addParametricCallback(0.98, () -> {
                    if (waitingForScore) return;
                    follower.pausePathFollowing();
                    lifty.setPower(0.0);
                    actionTimer.resetTimer();
                    waitingForScore = true;
                })
                .build();
        reload2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-88.0,88.0,Math.toRadians(135)), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(135), scorePose.getHeading())
                .addParametricCallback(0.98, () ->{
                    if (waitingForScore) return;
                    follower.pausePathFollowing();
                    lifty.setPower(-1.0);
                    actionTimer.resetTimer();
                    waitingForScore = true;
                })
                .build();
        pickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(144-81.5,29.0), pickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        finish = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(144-83,35, Math.toRadians(90))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(90))
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(move);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(score1, true);
                    intake.setPower(0.0);
                    back.setPosition(0.0);
                    launcher.setPower(0.60);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && !waitingForScore) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    follower.followPath(reload, true);
                    intake.setPower(0.0);
                    back.setPosition(0.5);
                    launcher.setPower(0.68);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload2, true);
                    intake.setPower(1.0);
                    back.setPosition(0.0);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload, true);
                    intake.setPower(0.0);
                    back.setPosition(0.5);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload2, true);
                    intake.setPower(1.0);
                    back.setPosition(0.0);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(pickup, true);
                    lifty.setPower(0.0);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(score2, true);
                    launcher.setPower(0.67);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload, true);
                    lifty.setPower(0.0);
                    intake.setPower(0.0);
                    back.setPosition(0.5);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload2, true);
                    intake.setPower(1.0);
                    lifty.setPower(-1.0);
                    back.setPosition(0.0);
                    setPathState(10);
                }
            case 10:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload, true);
                    intake.setPower(0.0);
                    back.setPosition(0.5);
                    setPathState(11);
                }
            case 11:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(reload2, true);
                    intake.setPower(1.0);
                    back.setPosition(0.0);
                    setPathState(12);
                }
            case 12:
                if (!follower.isBusy() && !waitingForScore) {
                    follower.followPath(finish, true);
                    intake.setPower(0.0);
                    launcher.setPower(0.0);
                    back.setPosition(0.5);
                    lifty.setPower(0.0);
                }
            case 13:
                if (!follower.isBusy() && !waitingForScore) {
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (waitingForScore && actionTimer.getElapsedTimeSeconds() > 1.0) {
            waitingForScore = false;
            follower.resumePathFollowing();
        }
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        launcher = hardwareMap.get(DcMotor.class, "motor4");
        lifty = hardwareMap.get(DcMotor.class, "motor5");
        intake = hardwareMap.get(DcMotor.class, "motor6");
        back = hardwareMap.get(Servo.class, "servo0");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(144-84.0,8.5,Math.toRadians(90)));
    }

    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
