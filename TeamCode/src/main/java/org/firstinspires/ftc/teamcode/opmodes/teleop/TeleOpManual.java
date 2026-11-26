package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * TeleOp ROJO con Pedro 3 + auto-aim de torreta. Arranca en la pose final del AutoPath (o en START si no hubo auto).
 *  Sticks: manejo FIELD CENTRIC (mover cualquier stick cancela el auto-drive / el hold)
 *  dpad_right: ir a (48, 24) + cell derecha  |  dpad_left: ir a (48, 120) + cell izquierda (los dos regresan a auto-aim)
 *  dpad_up / dpad_down: torreta MANUAL de emergencia (+/- TURRET_STEP_DEG), apaga el auto-aim
 *  A: shooter on/off  |  RB: ráfaga (solo a velocidad)  |  RT: intake  |  LT: escupir  |  Y: flower
 */
@Configurable
@TeleOp(name = "TeleOpManual", group = "teleop")
public class TeleOpManual extends OpMode {
    public static double SHOOT_TPS      = 1060; // tuneado en ShooterTuner
    public static double HOOD_POS       = 0.25; // tuneado en ShooterTuner
    public static double SHOOT_MS       = 800;
    public static double FLOWER_SAFE_MS = 300;  // tiempo para que la flower llegue a SHOOT_POS
    public static double STRAFE_MULT    = 1.1;
    public static double STICK_DB       = 0.08; // stick arriba de esto cancela el auto-drive
    public static double TURRET_STEP_DEG = 10;  // torreta manual: grados por toque (+ = izquierda)
    // Dirección del campo (frame de Pedro) que es "adelante" para el driver: 0, 90, 180 o 270
    public static double FIELD_FORWARD_DEG = 0;

    // Poses en frame de Pedro (in, grados). Headings iguales a los de AutoPath (de frente al HIVE)
    public static double START_X = 48, START_Y = 9,   START_HEADING_DEG = 90;  // igual que el start del AutoPath
    public static double DOWN_X  = 48, DOWN_Y  = 24,  DOWN_HEADING_DEG  = 90;  // cell derecha
    public static double UP_X    = 48, UP_Y    = 120, UP_HEADING_DEG    = 270; // cell izquierda

    private final PoseFactory poses = PoseFactory.degrees();

    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ShooterSubsystem shooter;
    private GateSubsystem gate;
    private HoodSubsystem hood;
    private TurretSubsystem turret;
    private FlowerSubsystem flower;

    private boolean shooterOn = false;
    private boolean lastA = false, lastY = false;
    private boolean lastDpadDown = false, lastDpadUp = false, lastDpadLeft = false, lastDpadRight = false;
    private boolean bursting = false;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private Pose startPose;
    private boolean manualTurret = false;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        intake   = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        shooter  = new ShooterSubsystem(hardwareMap);
        gate     = new GateSubsystem(hardwareMap);
        hood     = new HoodSubsystem(hardwareMap);
        turret   = new TurretSubsystem(hardwareMap);
        flower   = new FlowerSubsystem(hardwareMap);

        turret.setAlliance(false); // obligatorio: sin esto goalX/goalY quedan en 0
        turret.setRightCell(true);

        follower = Constants.create(hardwareMap);
        // Hereda la pose del AutoPath; si no hay (TeleOp sin auto antes), usa START
        boolean fromAuto = PoseStorage.pose != null;
        startPose = fromAuto ? PoseStorage.pose : poses.of(START_X, START_Y, START_HEADING_DEG);
        follower.setPose(startPose);
        follower.update();
        telemetry.addData("Pose inicial", fromAuto ? "HEREDADA (auto o TeleOp anterior)" : "START");
        telemetry.addData("x / y / h inicial", "%.1f / %.1f / %.1f",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Se vuelve a escribir: al crear el follower el Pinpoint recalibra el IMU y puede ignorar el setPose del init
        follower.setPose(startPose);
        follower.update();
        gate.close();
        flower.up();
    }

    @Override
    public void loop() {
        // ---------- Drive ----------
        double y  = -gamepad1.left_stick_y;
        double x  = -gamepad1.left_stick_x * STRAFE_MULT; // Pedro: lateral + = izquierda
        double rx =  -gamepad1.right_stick_x;
        boolean sticks = Math.abs(y) > STICK_DB || Math.abs(x) > STICK_DB || Math.abs(rx) > STICK_DB;

        if (gamepad1.dpad_right && !lastDpadRight) {
            goTo(DOWN_X, DOWN_Y, DOWN_HEADING_DEG, true);
        } else if (gamepad1.dpad_left && !lastDpadLeft) {
            goTo(UP_X, UP_Y, UP_HEADING_DEG, false);
        }
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft  = gamepad1.dpad_left;

        // Torreta manual de emergencia (parte del ángulo actual, sin brincos; el subsystem respeta la ventana)
        if (gamepad1.dpad_up && !lastDpadUp) {
            manualTurret = true;
            turret.setAngleDeg(turret.getTargetDeg() + TURRET_STEP_DEG);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            manualTurret = true;
            turret.setAngleDeg(turret.getTargetDeg() - TURRET_STEP_DEG);
        }
        lastDpadUp   = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // Auto-drive (FOLLOW, y HOLD al llegar) hasta que el driver toque los sticks
        Follower.Mode mode = follower.mode();
        boolean autoDrive = mode == Follower.Mode.FOLLOW || mode == Follower.Mode.HOLD;
        if (!autoDrive || sticks) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    y, x, rx, follower.pose().heading(), -Math.toRadians(FIELD_FORWARD_DEG));
            follower.manual(powers);
        }

        follower.update();
        Pose pose = follower.pose();
        PoseStorage.save(pose); // si reinicias el TeleOp, sigue desde aquí

        // ---------- Turret (auto-aim, salvo que esté en manual) ----------
        if (!manualTurret) turret.aim(pose.x(), pose.y(), pose.heading(), follower.velocity().omega);

        // ---------- Shooter ----------
        if (gamepad1.a && !lastA) shooterOn = !shooterOn;
        lastA = gamepad1.a;

        if (shooterOn) shooter.setTargetTps(SHOOT_TPS);
        else           shooter.stop();
        shooter.update();

        hood.setPosition(HOOD_POS);

        // ---------- Ráfaga: flower a SHOOT, espera, abre gate, regresa flower a UP ----------
        if (!bursting && gamepad1.right_bumper && shooter.isAtSpeed()) {
            bursting = true;
            flower.shoot();
            shootTimer.reset();
        }
        if (bursting && shootTimer.milliseconds() >= FLOWER_SAFE_MS + SHOOT_MS) {
            bursting = false;
            flower.up();
        }

        // ---------- Flower (no se toca durante la ráfaga) ----------
        if (!bursting && gamepad1.y && !lastY) flower.toggle();
        lastY = gamepad1.y;

        // ---------- Intake / Transfer / Gate ----------
        if (bursting && shootTimer.milliseconds() >= FLOWER_SAFE_MS) {
            gate.open();
            transfer.feed();
            intake.in();
        } else if (bursting) {
            gate.close();          // esperando a que la flower deje libre el shooter
            transfer.stop();
            intake.stop();
        } else {
            gate.close();
            if (gamepad1.right_trigger > 0.05) {
                intake.in();
                transfer.stop();   // el transfer no se mueve mientras hay intake
            } else if (gamepad1.left_trigger > 0.05) {
                intake.out();
                transfer.reverse();
            } else {
                intake.stop();
                transfer.stop();
            }
        }

        telemetry.addData("Drive", mode);
        telemetry.addData("x / y / h", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Cell", turret.isRightCell() ? "DERECHA" : "IZQUIERDA");
        telemetry.addData("Torreta", manualTurret ? "MANUAL" : "AUTO-AIM");
        telemetry.addData("Turret deg / dist", "%.1f / %.1f", turret.getTargetDeg(), turret.getDistanceIn());
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("FW Target / Actual", "%.0f / %.0f", shooter.getTargetTps(), shooter.getVelocity());
        telemetry.addData("At Speed", shooter.isAtSpeed());
        telemetry.addData("Flower", flower.getState());
        telemetry.addData("Gate", gate.isOpen() ? "OPEN" : "CLOSED");
        telemetry.update();
    }

    /** Path recto desde la pose actual; se construye solo en el flanco del botón. */
    private void goTo(double x, double y, double headingDeg, boolean rightCell) {
        turret.setRightCell(rightCell);
        manualTurret = false; // ir a una posición de tiro regresa al auto-aim
        Pose target = poses.of(x, y, headingDeg);
        // Ya estás encima del punto: un path de largo ~0 da tangente NaN, mejor solo sostener ahí
        if (follower.pose().distance(target) < 1.0) follower.hold(target);
        else follower.follow(Paths.line(follower.pose(), target).constant(target));
    }

    @Override
    public void stop() {
        follower.stop();
        shooter.stop();
        shooter.update();
        intake.stop();
        transfer.stop();
    }
}
