package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motor0");
        c.frontRightName.set("motor2");
        c.backLeftName.set("motor1");
        c.backRightName.set("motor3");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.3);
        c.yPodOffset.set(-6.75);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1576513525705069);
                Controller secondaryTranslationalForward = Controller.proportional(0.05824796225307848);
                Controller primaryTranslationalLateral = Controller.proportional(0.1716186056961858);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0634084890711398);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01474492845471532));
                c.brake.set(Controller.proportionalFeedforward(0.012533189186508021));

                c.headingFeedback.set(Controller.proportional(3.019439226070615));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.037942099091677485, 0.005178188729764574));

                c.linearBrakeCoefficients.set(Matrix.diag(0.054274237563256306, 0.03703465317581217));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001075779995902837, 0.0014627664827851733));

                c.maxAchievableForwardVelocity.set(67.53298119638623);
                c.maxAchievableStrafeVelocity.set(59.82547302406083);
                c.naturalForwardDeceleration.set(38.52531489519387);
                c.naturalStrafeDeceleration.set(45.71532262489677);
            }
    );
}