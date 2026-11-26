package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Hood con 2 servos espejeados. Rango útil validado: 0.2 (abajo) a 1.0 (arriba). */
@Configurable
public class HoodSubsystem {
    public static double MIN_POS = 0.2;  // abajo (debajo de esto empuja el tope)
    public static double MAX_POS = 0.8;  // arriba (bajar a 0.98 si zumba)

    private final Servo left, right;
    private double pos = 0.0;
    private boolean written = false;

    public HoodSubsystem(HardwareMap hw) {
        left  = hw.get(Servo.class, "left_hood");
        right = hw.get(Servo.class, "right_hood");
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE); // 0 lógico = 1 físico
    }

    public void setPosition(double h) {
        h = Range.clip(h, MIN_POS, MAX_POS);
        if (written && Math.abs(h - pos) < 1e-3) return;
        left.setPosition(h);
        right.setPosition(h);
        pos = h;
        written = true;
    }

    public void down() { setPosition(MIN_POS); }
    public double getPosition() { return pos; }
}
