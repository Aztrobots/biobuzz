package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FL");
                c.backLeftName.set("BL");
                c.frontRightName.set("FR");
                c.backRightName.set("BR");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            });

    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("BR");
        c.yPodName.set("FL");
        c.imuName.set("imu");
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.REVERSE);

        c.forwardTicksToInches.set(0.002005515167);
        c.strafeTicksToInches.set(0.001984455102);

        c.xPodOffset.set(-7.236);   // forward pod: 183.783 mm a la izquierda
        c.yPodOffset.set(-0.252);  // strafe pod: 6.4 mm atrás

        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.31552971003220026);
                Controller secondaryTranslationalForward = Controller.proportional(0.11657979674776799);
                Controller primaryTranslationalLateral = Controller.proportional(0.4846733544428042);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17907385375606877);

                c.forwardTranslational.set(
                        Controller.piecewise(secondaryTranslationalForward)
                                .put(2.5, primaryTranslationalForward)
                );
                c.strafeTranslational.set(
                        Controller.piecewise(secondaryTranslationalLateral)
                                .put(2.5, primaryTranslationalLateral)
                );

                c.coast.set(Controller.proportionalFeedforward(0.013286381799907175));
                c.brake.set(Controller.proportionalFeedforward(0.011293424529921098));

                c.headingFeedback.set(Controller.proportional(6.213198583140544));
                c.headingBrakeCoefficients.set(
                        Vector2D.cartesian(0.05092194840123294, 0.003595529773604995)
                );

                c.linearBrakeCoefficients.set(
                        Matrix.diag(0.059068982772272516, 0.03844171134078416)
                );
                c.quadraticBrakeCoefficients.set(
                        Matrix.diag(0.0014903678397327079, 0.0022999031900255316)
                );

                c.maxAchievableForwardVelocity.set(71.54611106818068);
                c.maxAchievableStrafeVelocity.set(56.816069014094225);
                c.naturalForwardDeceleration.set(40.11977103042037);
                c.naturalStrafeDeceleration.set(79.80594429227354);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
}