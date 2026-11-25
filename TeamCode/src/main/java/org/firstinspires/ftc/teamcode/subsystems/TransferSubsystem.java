package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class TransferSubsystem {
    public static double FEED_POWER    = 0.45;
    public static double REVERSE_POWER = -1.0;
    public static double HOLD_POWER    = -0.65; // reversa suave: rechaza pelotas mientras se hace intake
    public static boolean REVERSED = true;

    private final DcMotorEx motor;
    private double lastPower = -2.0;

    public TransferSubsystem(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "transfer");
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void feed()    { setPower(FEED_POWER); }
    public void reverse() { setPower(REVERSE_POWER); }
    public void hold()    { setPower(HOLD_POWER); }
    public void stop()    { setPower(0.0); }

    public void setPower(double p) {
        if (Math.abs(p - lastPower) < 1e-3) return;
        motor.setPower(p);
        lastPower = p;
    }
}