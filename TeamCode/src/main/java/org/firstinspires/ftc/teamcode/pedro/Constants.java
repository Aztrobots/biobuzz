package org.firstinspires.ftc.teamcode.pedro;

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
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.backLeftName.set("BL");
        c.frontRightName.set("FR");
        c.backRightName.set("BR");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.2329);
        c.yPodOffset.set(-0.69);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward   = Controller.proportional(0.4233752714890233);
        Controller secondaryTranslationalForward = Controller.proportional(0.1564258500195892);
        Controller primaryTranslationalLateral   = Controller.proportional(0.7175591790881266);
        Controller secondaryTranslationalLateral = Controller.proportional(0.2651189431386735);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.012403603640606525));
        c.brake.set(Controller.proportionalFeedforward(0.010543063094515547));

        c.headingFeedback.set(Controller.proportional(3.320950188628473));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06260319633255379, 0.005867670663905015));

        c.linearBrakeCoefficients.set(Matrix.diag(0.09450378558456209, 0.06429807590546179));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017660983073149899, 0.0017155714439221147));

        c.maxAchievableForwardVelocity.set(78.43036403031576);
        c.maxAchievableStrafeVelocity.set(61.125592073399766);
        c.naturalForwardDeceleration.set(34.32648004655617);
        c.naturalStrafeDeceleration.set(78.51115410718896);

        // ── Anti-vibración en hold (solo afecta hold, no el seguimiento de paths) ──
        c.holdPointTranslationalScaling.set(0.25);  // default 0.45
        c.holdPointHeadingScaling.set(0.20);        // default 0.35
        c.minCorrectionDistance.set(0.2);           // deadband en in; default 0.001
    });
}