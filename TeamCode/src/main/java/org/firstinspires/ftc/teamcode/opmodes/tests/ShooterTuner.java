package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * A: prender/apagar flywheel.
 * RT: solo intake.
 * RB (mantener): disparo = gate abierto + transfer + intake. RB tiene prioridad sobre RT.
 * Dpad arriba/abajo: hood +/- HOOD_STEP.
 * kF, kP, etc. se editan en ShooterSubsystem desde Panels.
 */
@Configurable
@TeleOp(name = "ShooterTuner", group = "test")
public class ShooterTuner extends OpMode {
    public static double TARGET_TPS = 1060;
    public static double HOOD_POS   = 0.25;
    public static double HOOD_STEP  = 0.05;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private GateSubsystem gate;
    private HoodSubsystem hood;
    private TelemetryManager panels;

    private boolean on = false;
    private boolean lastA = false;
    private boolean lastUp = false, lastDown = false;

    @Override
    public void init() {
        shooter  = new ShooterSubsystem(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        gate     = new GateSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        panels   = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        gate.close();
        hood.setPosition(HOOD_POS);
    }

    @Override
    public void loop() {
        // ---------- Flywheel ----------
        if (gamepad1.a && !lastA) on = !on;
        lastA = gamepad1.a;

        if (on) shooter.setTargetTps(TARGET_TPS);
        else    shooter.stop();
        shooter.update();

        // ---------- Hood ----------
        if (gamepad1.dpad_up && !lastUp)     HOOD_POS += HOOD_STEP;
        if (gamepad1.dpad_down && !lastDown) HOOD_POS -= HOOD_STEP;
        lastUp   = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        HOOD_POS = Math.max(0.0, Math.min(1.0, HOOD_POS)); // rango válido de servo
        hood.setPosition(HOOD_POS);

        // ---------- Gate / Transfer / Intake ----------
        boolean shooting = gamepad1.right_bumper;
        boolean intaking = gamepad1.right_trigger > 0.05;

        if (shooting) {
            gate.open();
            transfer.feed();
            intake.in();
        } else if (intaking) {
            gate.close();
            transfer.stop();
            intake.in();
        } else {
            gate.close();
            transfer.stop();
            intake.stop();
        }

        panels.addData("Target TPS", shooter.getTargetTps());
        panels.addData("Actual TPS", shooter.getVelocity());
        panels.addData("Error", shooter.getTargetTps() - shooter.getVelocity());
        panels.addData("Power", shooter.getPower());
        panels.addData("At Speed", shooter.isAtSpeed());
        panels.addData("Hood pedido / real", String.format("%.2f / %.2f", HOOD_POS, hood.getPosition()));
        panels.addData("Gate", gate.isOpen() ? "OPEN" : "CLOSED");
        panels.addData("Modo", shooting ? "SHOOT" : intaking ? "INTAKE" : "IDLE");
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        gate.close();
        shooter.stop();
        shooter.update();
        intake.stop();
        transfer.stop();
    }
}