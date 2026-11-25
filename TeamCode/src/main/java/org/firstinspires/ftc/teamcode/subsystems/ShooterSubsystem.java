package org.firstinspires.ftc.teamcode.subsystems;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class ShooterSubsystem {
    public static double MAX_TPS = 2500.0;
    public static double kF = 0.000458;
    public static double kP = 0.005;
    public static double BANG_BANG_THRESHOLD = 250; // TPS: arriba de esto, power = 1
    public static double AT_SPEED_TOL = 40;         // TPS
    public static double NOMINAL_VOLTAGE = 12.0;
    public static long   VOLTAGE_PERIOD_MS = 250;
    public static boolean REVERSED = true;

    private final DcMotorEx motor;
    private final VoltageSensor battery;

    private double targetTps = 0.0;
    private double velocity = 0.0;
    private double lastPower = -2.0; // sentinel
    private double voltage = NOMINAL_VOLTAGE;
    private long lastVoltageMs = 0;

    public ShooterSubsystem(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "fwl");
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        battery = hw.voltageSensor.iterator().next();
    }

    public void setTargetTps(double tps) { targetTps = Range.clip(tps, 0.0, MAX_TPS); }
    public void stop() { targetTps = 0.0; }

    /** Llamar una vez por loop. */
    public void update() {
        velocity = motor.getVelocity();

        long now = System.currentTimeMillis();
        if (now - lastVoltageMs >= VOLTAGE_PERIOD_MS) {
            double v = battery.getVoltage();
            if (v > 6.0) voltage = v;
            lastVoltageMs = now;
        }

        double power;
        if (targetTps <= 0.0) {
            power = 0.0;
        } else {
            double error = targetTps - velocity;
            if (error > BANG_BANG_THRESHOLD) {
                power = 1.0;
            } else {
                power = kF * targetTps * (NOMINAL_VOLTAGE / voltage) + kP * error;
            }
        }
        power = Range.clip(power, 0.0, 1.0); // nunca frenar activamente el flywheel

        if (Math.abs(power - lastPower) > 0.002) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public boolean isAtSpeed() {
        return targetTps > 0.0 && Math.abs(targetTps - velocity) < AT_SPEED_TOL;
    }

    public double getVelocity()  { return velocity; }
    public double getTargetTps() { return targetTps; }
    public double getPower()     { return Math.max(lastPower, 0.0); }
}
