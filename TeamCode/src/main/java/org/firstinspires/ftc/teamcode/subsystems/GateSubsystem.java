package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class GateSubsystem {
    public static double BLOCK_POS = 0.5;
    public static double OPEN_POS  = 0.8;

    private final Servo servo;

    private boolean open = false;
    private boolean written = false;

    public GateSubsystem(HardwareMap hw) {
        servo = hw.get(Servo.class, "gate");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void open()  { set(true); }
    public void close() { set(false); }
    public boolean isOpen() { return open; }

    private void set(boolean o) {
        if (written && o == open) return;
        servo.setPosition(o ? OPEN_POS : BLOCK_POS);
        open = o;
        written = true;
    }
}
