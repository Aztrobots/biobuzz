package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class IntakeSubsystem {
    public static double IN_POWER  = 1.0;
    public static double OUT_POWER = -1.0;
    // Motores encontrados: uno debe ir invertido para que ambos metan pieza
    public static boolean REVERSE_INTAKE  = false;
    public static boolean REVERSE_INTAKE1 = true;

    private final DcMotorEx m0, m1;
    private double lastPower = -2.0; // sentinel (no NaN)

    public IntakeSubsystem(HardwareMap hw) {
        m0 = hw.get(DcMotorEx.class, "intake");
        m1 = hw.get(DcMotorEx.class, "intake1");
        m0.setDirection(REVERSE_INTAKE  ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m1.setDirection(REVERSE_INTAKE1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{m0, m1}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void in()   { setPower(IN_POWER); }
    public void out()  { setPower(OUT_POWER); }
    public void stop() { setPower(0.0); }

    public void setPower(double p) {
        if (Math.abs(p - lastPower) < 1e-3) return; // evita writes redundantes al bus
        m0.setPower(p);
        m1.setPower(p);
        lastPower = p;
    }

    public double getPower() { return Math.max(lastPower, -1.0); }
}
