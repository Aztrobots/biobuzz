package org.firstinspires.ftc.teamcode.subsystems;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class TurretSubsystem {
    // Map
    public static double SERVO_TRAVEL_DEG = 268.0; // recorrido total de 0 a 1 (medido: TEST_DEG 90 -> 67° reales)
    public static double CENTER_POS = 0.5;         // torreta al centro
    // Hacia dónde dispara el shooter con la torreta en CENTER_POS, respecto al frente del robot (heading de Pedro).
    // 180 = dispara hacia la parte trasera del robot; 0 = hacia el frente.
    public static double SHOOTER_FACING_DEG = 0.0;
    public static boolean REVERSE_B = false;       // true si turret1 quedó espejeado
    // false: +grados = antihorario visto desde arriba (pos baja). true si la torreta gira al revés.
    public static boolean INVERT = false;
    // Ventana de software alrededor del centro (por seguridad; el cableado limita pos a <= 0.87)
    public static double MAX_LEFT_DEG  = 90.0;     // +
    public static double MAX_RIGHT_DEG = 90.0;     // −

    // Smoothing por distancia
    public static double TAU_NEAR_S = 0.15;      // cerca -> más filtro
    public static double TAU_FAR_S  = 0.04;      // lejos -> más reactivo
    public static double DIST_NEAR_IN = 24.0;
    public static double DIST_FAR_IN  = 110.0;
    public static double DEADBAND_DEG = 0.3;

    // Compensación de giro: adelanta la torreta ω·OMEGA_LEAD_S para cubrir el lag del servo. 0 = desactivado
    public static double OMEGA_LEAD_S = 0.05;

    //Goals rojos
    public static double RED_RIGHT_X = 60;
    public static double RED_RIGHT_Y = 59;
    public static double RED_LEFT_X  = 82;
    public static double RED_LEFT_Y  = 88;
    private static final double FIELD = 141.5;

    private final Servo servoA, servoB;

    private boolean isBlue = false;
    private boolean rightCell = true;
    private double goalX = 0.0, goalY = 0.0;

    private boolean bearingInit = false;
    private double filteredBearing = 0.0;        // rad, frame de campo
    private long lastAimNs = 0;

    private double targetDeg = 0.0;
    private double writtenDeg = 0.0;
    private boolean hasWritten = false;
    private double distanceIn = 0.0;
    private double servoPos = 0.5;

    public TurretSubsystem(HardwareMap hw) {
        servoA = hw.get(Servo.class, "turret");
        servoB = hw.get(Servo.class, "turret1");
        servoB.setDirection(REVERSE_B ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /** Llamar en init() de cada OpMode. */
    public void setAlliance(boolean isBlue) {
        this.isBlue = isBlue;
        updateGoal();
    }

    /** true = cell derecha, false = cell izquierda. */
    public void setRightCell(boolean right) {
        if (right == rightCell) return;
        rightCell = right;
        updateGoal();
    }

    public void toggleCell() { setRightCell(!rightCell); }

    private void updateGoal() {
        double x = rightCell ? RED_RIGHT_X : RED_LEFT_X;
        double y = rightCell ? RED_RIGHT_Y : RED_LEFT_Y;
        goalX = isBlue ? FIELD - x : x;
        goalY = isBlue ? FIELD - y : y;
        bearingInit = false; // cambio de goal: sin lag del filtro
    }

    /** Modo manual: ángulo relativo al centro de la torreta, sin filtro. */
    public void setAngleDeg(double deg) {
        bearingInit = false;
        command(deg);
    }

    /**
     * Auto-aim. Pose en pulgadas, heading en radianes, omega en rad/s (frame de Pedro).
     * Filtra solo el bearing de campo (ruido de odometría); el heading se resta sin filtrar
     * y adelantado por omega para que el servo no llegue tarde cuando el robot gira.
     */
    public void aim(double x, double y, double headingRad, double omegaRadS) {
        double dx = goalX - x;
        double dy = goalY - y;
        distanceIn = Math.hypot(dx, dy);
        if (distanceIn < 1.0) return;

        double rawBearing = Math.atan2(dy, dx);
        long now = System.nanoTime();

        if (!bearingInit) {
            filteredBearing = rawBearing;
            bearingInit = true;
        } else {
            double dt = (now - lastAimNs) / 1e9;
            double t = Range.clip((distanceIn - DIST_NEAR_IN) / (DIST_FAR_IN - DIST_NEAR_IN), 0.0, 1.0);
            double tau = Math.max(TAU_NEAR_S + (TAU_FAR_S - TAU_NEAR_S) * t, 1e-3);
            double alpha = 1.0 - Math.exp(-dt / tau);
            filteredBearing = AngleUnit.normalizeRadians(
                    filteredBearing + alpha * AngleUnit.normalizeRadians(rawBearing - filteredBearing));
        }
        lastAimNs = now;

        if (Double.isNaN(omegaRadS)) omegaRadS = 0.0;
        double shooterHeading = headingRad + omegaRadS * OMEGA_LEAD_S + Math.toRadians(SHOOTER_FACING_DEG);
        command(Math.toDegrees(AngleUnit.normalizeRadians(filteredBearing - shooterHeading)));
    }

    private void command(double deg) {
        // Ventana de software; sin wrap: un objetivo fuera de la ventana se queda en el tope más cercano
        targetDeg = Range.clip(deg, -MAX_RIGHT_DEG, MAX_LEFT_DEG);
        if (hasWritten && Math.abs(targetDeg - writtenDeg) < DEADBAND_DEG) return;

        writtenDeg = targetDeg;
        hasWritten = true;
        double sign = INVERT ? -1.0 : 1.0;
        servoPos = Range.clip(CENTER_POS - sign * writtenDeg / SERVO_TRAVEL_DEG, 0.0, 1.0);
        servoA.setPosition(servoPos);
        servoB.setPosition(servoPos);
    }

    public double getServoPos() { return servoPos; }
    public double getGoalX()    { return goalX; }
    public double getGoalY()    { return goalY; }

    public boolean isRightCell()  { return rightCell; }
    public double getTargetDeg()  { return targetDeg; }
    public double getDistanceIn() { return distanceIn; }
}