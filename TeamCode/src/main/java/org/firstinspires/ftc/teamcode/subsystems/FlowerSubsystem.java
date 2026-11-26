package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Flower con 2 servos espejeados. UP = reposo (tapa el shooter), SHOOT = pose segura para disparar, DOWN = intake. */
@Configurable
public class FlowerSubsystem {
    public static double UP_POS    = 0.0;
    public static double SHOOT_POS = 0.3;   // no tapa el shooter
    public static double DOWN_POS  = 0.475;

    public enum State { UP, SHOOT, DOWN }

    private final Servo left, right;
    private State state = State.UP;
    private boolean written = false;

    public FlowerSubsystem(HardwareMap hw) {
        left  = hw.get(Servo.class, "left_flower");
        right = hw.get(Servo.class, "right_flower");
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE); // 0 lógico = 1 físico
    }

    public void up()     { set(State.UP); }
    public void shoot()  { set(State.SHOOT); }
    public void down()   { set(State.DOWN); }
    public void toggle() { set(state == State.DOWN ? State.UP : State.DOWN); }
    public boolean isUp() { return state == State.UP; }
    public State getState() { return state; }

    private void set(State s) {
        if (written && s == state) return;
        double pos = s == State.UP ? UP_POS : s == State.SHOOT ? SHOOT_POS : DOWN_POS;
        left.setPosition(pos);
        right.setPosition(pos);
        state = s;
        written = true;
    }
}