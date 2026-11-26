package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Prueba de torreta con el robot QUIETO (se puede empujar a mano).
 *  A: modo MANUAL (TEST_DEG)  /  B: modo AIM (auto-aim con la pose del Pinpoint)
 *  X: alterna cell derecha / izquierda
 *  Y: re-localiza en START_X / START_Y / START_H
 */
@Configurable
@TeleOp(name = "TurretTest", group = "test")
public class TurretTest extends OpMode {
    public static boolean IS_BLUE = false;
    public static double TEST_DEG = 0.0;
    public static double START_X = 48;
    public static double START_Y = 9;
    public static double START_H = 270; // grados

    private Follower follower;
    private TurretSubsystem turret;
    private TelemetryManager panels;
    private final PoseFactory pf = PoseFactory.degrees();

    private boolean aimMode = false;
    private boolean lastX = false, lastY = false;

    @Override
    public void init() {
        turret = new TurretSubsystem(hardwareMap);
        turret.setAlliance(IS_BLUE);
        follower = Constants.create(hardwareMap);
        follower.setPose(pf.of(START_X, START_Y, START_H));
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        if (gamepad1.a) aimMode = false;
        if (gamepad1.b) aimMode = true;
        if (gamepad1.x && !lastX) turret.toggleCell();
        if (gamepad1.y && !lastY) follower.setPose(pf.of(START_X, START_Y, START_H));
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        if (aimMode) turret.aim(pose.x(), pose.y(), pose.heading(), follower.velocity().omega);
        else         turret.setAngleDeg(TEST_DEG);

        double bearing = Math.toDegrees(Math.atan2(turret.getGoalY() - pose.y(), turret.getGoalX() - pose.x()));

        panels.addData("Modo", aimMode ? "AIM" : "MANUAL");
        panels.addData("Pose x / y / h", String.format("%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        panels.addData("Cell", turret.isRightCell() ? "DERECHA" : "IZQUIERDA");
        panels.addData("Goal x / y", String.format("%.1f / %.1f", turret.getGoalX(), turret.getGoalY()));
        panels.addData("Bearing campo (deg)", String.format("%.1f", bearing));
        panels.addData("Turret target (deg)", String.format("%.1f", turret.getTargetDeg()));
        panels.addData("Servo pos", String.format("%.3f", turret.getServoPos()));
        panels.addData("Dist (in)", String.format("%.1f", turret.getDistanceIn()));
        panels.update(telemetry);
    }
}