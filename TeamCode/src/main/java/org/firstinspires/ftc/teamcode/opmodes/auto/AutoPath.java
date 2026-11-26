package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
 * Auto ROJO — precarga + GARDEN + FLOWER + alliance balls + park.
 *
 * Flower: 0 (UP) todo el tiempo, 0.3 (SHOOT) solo durante el disparo,
 *         0.475 (DOWN) solo durante intoFlower.
 * Disparo: el robot llega con el heading del path, GIRA SOBRE SU EJE para quedar
 *          de frente al HIVE, dispara y (si el siguiente path lo pide) gira de regreso.
 *
 *  0. Shooter ON desde el start
 *  1. preLoad          -> giro a 90  -> SHOOT1 -> giro a 270
 *  INTAKE: desde intoGarden ya NO se apaga en todo el auto (intakeLatched)
 *  2. [intake ON, transfer quieto] toGarden, intoGarden -> espera (una sola entrada)
 *  3. outGarden -> shootingPose -> giro a 270 -> SHOOT2 -> afterShoot (y-6) -> giro a 90
 *  4. [flower DOWN + intake ON] intoFlower
 *  5. shootingPose1 (atrás, flower aún abajo) [flower UP] -> giro a 270 -> SHOOT3
 *  6. [intake ON] giro a 180 -> toAllianceBalls (heading fijo) -> espera (igual que GARDEN), outAllianceBalls, backToShootingPose [intake OFF]
 *                      -> giro a 90  -> SHOOT4
 *  7. shooter OFF -> park
 */
@Configurable
@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    // ---------- Tunables (Panels) ----------
    public static boolean IS_BLUE = false;
    public static double PATH_DELAY_MS     = 200;
    public static double PRELOAD_TPS       = 1080;
    public static double SHOOT_TPS         = 1080;
    public static double HOOD_POS          = 0.55;
    public static double SHOOT_MS          = 1000; // ráfaga de 4 POLLEN
    public static double SPINUP_TIMEOUT_MS = 1500; // dispara aunque isAtSpeed() no llegue
    public static double FLOWER_SAFE_MS    = 300;  // tiempo para que la flower llegue a SHOOT_POS
    public static double INTAKE_GARDEN_MS  = 500; // intake ON en el GARDEN

    // Giro sobre su eje en cada shooting pose
    public static double SHOOT_HEADING_LOW  = 90;   // en (48, 24): de frente al HIVE
    public static double SHOOT_HEADING_HIGH = 270;  // en (48, 120): de frente al HIVE
    public static double TURN_TOL_DEG       = 4;
    public static double TURN_TIMEOUT_MS    = 1500;

    // Cell por tiro: true = derecha, false = izquierda (la que esté ARRIBA en ese momento)
    // Default = la cell más cercana a cada shooting pose (~35-37 in)
    public static boolean SHOT1_RIGHT_CELL = true;   // (48, 24)   -> der 37 in / izq 63 in
    public static boolean SHOT2_RIGHT_CELL = false;  // (48, 120)  -> izq 35 in / der 61 in
    public static boolean SHOT3_RIGHT_CELL = false;  // (48, 120)
    public static boolean SHOT4_RIGHT_CELL = true;   // (47.7, 24)

    // ---------- Hardware ----------
    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ShooterSubsystem shooter;
    private GateSubsystem gate;
    private HoodSubsystem hood;
    private TurretSubsystem turret;
    private FlowerSubsystem flower;

    private String step = "INIT";
    private boolean intakeLatched = false; // true desde intoGarden: el intake ya no se apaga

    // ---------- Poses ----------
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose iNTAKEGARDEN = poseFactory.of(48, 7, 90); // START
    private final Pose preLoad = poseFactory.of(48, 24, 90);
    private final Pose toGardenStart = poseFactory.of(48, 24, 270);
    private final Pose toGarden = poseFactory.of(5.9307, 11.1588, 270);
    private final Pose intoGardenStart = poseFactory.of(5.9307, 11.1588, 270);
    private final Pose intoGarden = poseFactory.of(5, 13.5, 270);
    private final Pose backStart = poseFactory.of(5, 13.5, 270);
    private final Pose back = poseFactory.of(5.015, 12.9609, 270);
    private final Pose intoGarden1Start = poseFactory.of(5.015, 12.9609, 270);
    private final Pose intoGarden1 = poseFactory.of(5.1014, 12.5, 270);
    private final Pose back1Start = poseFactory.of(5.1014, 12.5, 270);
    private final Pose back1 = poseFactory.of(5.0751, 13.1579, 270);
    // Sale directo desde el final de intoGarden (sin back / intoGarden1 / back1)
    private final Pose outGardenStart = poseFactory.of(5, 11.5, 270);
    private final Pose outGarden = poseFactory.of(20.3351, 23.3, 90);
    private final Pose shootingPoseStart = poseFactory.of(20.3351, 23.3, 90);
    private final Pose shootingPose = poseFactory.of(48, 120, 90);
    private final Pose shootingPoseControl1 = poseFactory.of(12.321, 77.495, 0);
    // Después de SHOOT2: se aleja 6 in de la flor (sigue viendo a 270) antes de girar
    private final Pose afterShootStart = poseFactory.of(48, 120, 270);
    private final Pose afterShoot = poseFactory.of(48.5, 114, 270);
    private final Pose intoFlowerStart = poseFactory.of(48.5, 114, 90);
    private final Pose intoFlower = poseFactory.of(48.5, 126, 90);
    private final Pose shootingPose1Start = poseFactory.of(48.5, 128, 90);
    private final Pose shootingPose1 = poseFactory.of(48, 120.1, 90);
    // Heading 180: después de SHOOT3 gira sobre su eje a 180 y de ahí avanza sin girar
    private final Pose toAllianceBallsStart = poseFactory.of(48, 120.1, 180);
    private final Pose toAllianceBalls = poseFactory.of(26.6289, 129.7874, -180);
    private final Pose toAllianceBallsControl1 = poseFactory.of(32.9146, 119.9901, 0);
    private final Pose outAllianceBallsStart = poseFactory.of(26.6289, 129.7874, 180);
    private final Pose outAllianceBalls = poseFactory.of(26.8187, 122.8572, -90);
    private final Pose backToShootingPoseStart = poseFactory.of(26.8187, 122.8572, -90);
    private final Pose backToShootingPose = poseFactory.of(47.6521, 23.9605, -90);
    private final Pose backToShootingPoseControl1 = poseFactory.of(13.0092, 53.8628, 0);
    private final Pose park = poseFactory.of(10.8833, 94.3188, -80.1947);
    private final Pose parkControl1 = poseFactory.of(18.1143, 52.4788, 0);

    // ---------- Routine ----------
    public Command autoRoutine() {
        return sequential(
                // 0. Shooter ON desde el start (acelera mientras avanza a preLoad)
                instant(() -> shooter.setTargetTps(PRELOAD_TPS)),

                // 1. Precarga
                instant(() -> turret.setRightCell(SHOT1_RIGHT_CELL)),
                mark("preLoad"),       follow(follower, preLoad()),       delay(),
                turnTo(preLoad, SHOOT_HEADING_LOW),
                shoot("SHOOT1", PRELOAD_TPS),
                turnTo(preLoad, 270),

                // 2. GARDEN: intake ON desde que sale hacia el GARDEN
                instant(this::intakeOn),
                mark("toGarden"),      follow(follower, toGarden()),      delay(),
                mark("intoGarden"),    follow(follower, intoGarden()),
                instant(() -> intakeLatched = true),
                mark("intake GARDEN"),
                waitMs(INTAKE_GARDEN_MS),
                waitMs(INTAKE_GARDEN_MS),
                instant(this::intakeOff),                                 delay(),

                // 3. Sale del GARDEN y tiro 2 en (48, 120)
                mark("outGarden"),     follow(follower, outGarden()),     delay(),
                instant(() -> turret.setRightCell(SHOT2_RIGHT_CELL)),
                mark("shootingPose"),  follow(follower, shootingPose()),  delay(),
                turnTo(shootingPose, SHOOT_HEADING_HIGH),
                shoot("SHOOT2", SHOOT_TPS),
                mark("afterShoot"),    follow(follower, afterShoot()),    delay(),
                turnTo(afterShoot, 90),

                // 4. FLOWER (única vez que baja la flower)
                instant(() -> { flower.down(); intakeOn(); }),
                mark("intoFlower"),    follow(follower, intoFlower()),    delay(),delay(), delay(),delay(),

                // 5. Se hace para atrás con la flower ABAJO, ya afuera la sube, y tiro 3
                instant(() -> turret.setRightCell(SHOT3_RIGHT_CELL)),
                mark("shootingPose1"), follow(follower, shootingPose1()),
                instant(flower::up),
                instant(this::intakeOff),                                 delay(),
                turnTo(shootingPose1, SHOOT_HEADING_HIGH),
                shoot("SHOOT3", SHOOT_TPS),

                // 6. Alliance balls (flower en 0)
                instant(this::intakeOn),
                turnTo(shootingPose1, 180),   // gira sobre su eje ANTES de avanzar
                mark("toAllianceBalls"),  follow(follower, toAllianceBalls()),
                mark("intake ALLIANCE"),
                waitMs(INTAKE_GARDEN_MS),
                waitMs(INTAKE_GARDEN_MS),
                mark("outAllianceBalls"), follow(follower, outAllianceBalls()), delay(),
                instant(() -> turret.setRightCell(SHOT4_RIGHT_CELL)),
                mark("backToShootingPose"), follow(follower, backToShootingPose()),
                instant(this::intakeOff),                                 delay(),
                turnTo(backToShootingPose, SHOOT_HEADING_LOW),
                shoot("SHOOT4", SHOOT_TPS),

                // 7. Park
                instant(shooter::stop),
                mark("park"),          follow(follower, park()),
                mark("DONE")
        );
    }

    /** Intake metiendo pieza; el transfer no se mueve. */
    private void intakeOn()  { intake.in();   transfer.stop(); }
    /** Desde intoGarden (intakeLatched) solo para el transfer; el intake sigue prendido. */
    private void intakeOff() {
        if (!intakeLatched) intake.stop();
        transfer.stop();
    }

    /** Delay fijo entre paths. */
    private Command delay() {
        return waitMs(PATH_DELAY_MS);
    }

    /**
     * Giro sobre su eje: hold en la (x, y) del pose con el heading pedido,
     * termina al llegar a TURN_TOL_DEG o por timeout.
     */
    private Command turnTo(Pose at, double headingDeg) {
        Pose target = poseFactory.of(at.x(), at.y(), headingDeg);
        return sequential(
                mark("turn " + (int) headingDeg),
                instant(() -> follower.hold(target)),
                race(
                        waitUntil(() -> Math.abs(Math.toDegrees(AngleUnit.normalizeRadians(
                                follower.pose().heading() - target.heading()))) < TURN_TOL_DEG),
                        waitMs(TURN_TIMEOUT_MS)
                )
        );
    }

    private Command mark(String name) {
        return instant(() -> step = name);
    }

    /**
     * Flower a pose segura (0.3) + TPS, espera flower y velocidad en paralelo,
     * ráfaga, y regresa la flower a 0.
     */
    private Command shoot(String name, double tps) {
        return sequential(
                mark(name),
                instant(() -> { flower.shoot(); shooter.setTargetTps(tps); }),
                parallel(
                        waitMs(FLOWER_SAFE_MS),
                        race(waitUntil(shooter::isAtSpeed), waitMs(SPINUP_TIMEOUT_MS))
                ),
                instant(() -> { gate.open(); transfer.feed(); intake.in(); }),
                waitMs(SHOOT_MS),
                instant(() -> { gate.close(); transfer.stop(); flower.up(); if (!intakeLatched) intake.stop(); })
        );
    }

    // ---------- OpMode ----------
    @Override
    public void runOpMode() {
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

        turret.setAlliance(IS_BLUE);
        turret.setAngleDeg(0.0); // centro: STARTING CONFIGURATION
        hood.setPosition(HOOD_POS);
        gate.close();
        flower.up();

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(iNTAKEGARDEN);
        follower.update();

        // Paths y comandos construidos en init, no después del start
        Command routine = autoRoutine();

        telemetry.addLine("Listo");
        telemetry.update();

        waitForStart();
        PoseStorage.pose = null; // borra la pose del partido anterior para que el filtro acepte la nueva
        schedule(routine);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose pose = follower.pose();
            PoseStorage.save(pose); // el TeleOp arranca desde aquí
            turret.aim(pose.x(), pose.y(), pose.heading(), follower.velocity().omega);
            shooter.update();

            telemetry.addData("Step", step);
            telemetry.addData("x / y / h", "%.1f / %.1f / %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("FW Target / Actual", "%.0f / %.0f",
                    shooter.getTargetTps(), shooter.getVelocity());
            telemetry.addData("At Speed", shooter.isAtSpeed());
            telemetry.addData("Turret deg / dist", "%.1f / %.1f",
                    turret.getTargetDeg(), turret.getDistanceIn());
            telemetry.addData("Flower", flower.getState());
            if (follower.currentPath() != null) {
                telemetry.addData("Dist remaining", follower.distanceToEndpoint());
            }
            telemetry.update();
        }

        shooter.stop();
        shooter.update();
        intake.stop();
        transfer.stop();
    }

    // ---------- Paths ----------
    public Path preLoad() {
        return Paths.line(iNTAKEGARDEN, preLoad).linear(iNTAKEGARDEN, preLoad);
    }

    public Path toGarden() {
        return Paths.line(toGardenStart, toGarden).linear(toGardenStart, toGarden);
    }

    public Path intoGarden() {
        return Paths.line(intoGardenStart, intoGarden).linear(intoGardenStart, intoGarden);
    }

    public Path back() {
        return Paths.line(backStart, back).linear(backStart, back);
    }

    public Path intoGarden1() {
        return Paths.line(intoGarden1Start, intoGarden1).linear(intoGarden1Start, intoGarden1);
    }

    public Path back1() {
        return Paths.line(back1Start, back1).linear(back1Start, back1);
    }

    public Path outGarden() {
        return Paths.line(outGardenStart, outGarden).linear(outGardenStart, outGarden);
    }

    public Path shootingPose() {
        return Paths.curve(shootingPoseStart, shootingPoseControl1, shootingPose).linear(shootingPoseStart, shootingPose);
    }

    public Path afterShoot() {
        return Paths.line(afterShootStart, afterShoot).constant(afterShoot);
    }

    public Path intoFlower() {
        return Paths.line(intoFlowerStart, intoFlower).linear(intoFlowerStart, intoFlower);
    }

    public Path shootingPose1() {
        return Paths.line(shootingPose1Start, shootingPose1).linear(shootingPose1Start, shootingPose1);
    }

    public Path toAllianceBalls() {
        return Paths.curve(toAllianceBallsStart, toAllianceBallsControl1, toAllianceBalls).constant(toAllianceBalls);
    }

    public Path outAllianceBalls() {
        return Paths.line(outAllianceBallsStart, outAllianceBalls).linear(outAllianceBallsStart, outAllianceBalls);
    }

    public Path backToShootingPose() {
        return Paths.curve(backToShootingPoseStart, backToShootingPoseControl1, backToShootingPose).linear(backToShootingPoseStart, backToShootingPose);
    }

    public Path park() {
        return Paths.curve(backToShootingPose, parkControl1, park).reverseTangent();
    }
}