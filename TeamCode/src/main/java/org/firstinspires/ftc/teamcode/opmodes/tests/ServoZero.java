package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoZero", group = "test")
public class ServoZero extends LinearOpMode {
    public static double TURRET_POS = 0.5;
    public static double HOOD_POS   = 0.2;  // 0.2 = abajo. right = 1 - left
    public static double FLOWER_POS = 0.0;  // right = 1 - left
    public static double GATE_POS   = 0.0;  // 0 = cerrado, 0.15 = abierto

    @Override
    public void runOpMode() {
        Servo turret      = hardwareMap.get(Servo.class, "turret");
        Servo turret1     = hardwareMap.get(Servo.class, "turret1");
        Servo hoodRight   = hardwareMap.get(Servo.class, "right_hood");
        Servo hoodLeft    = hardwareMap.get(Servo.class, "left_hood");
        Servo flowerRight = hardwareMap.get(Servo.class, "right_flower");
        Servo flowerLeft  = hardwareMap.get(Servo.class, "left_flower");
        Servo gate        = hardwareMap.get(Servo.class, "gate");

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            turret.setPosition(TURRET_POS);
            turret1.setPosition(TURRET_POS);
            hoodLeft.setPosition(HOOD_POS);
            hoodRight.setPosition(1.0 - HOOD_POS);
            flowerLeft.setPosition(FLOWER_POS);
            flowerRight.setPosition(1.0 - FLOWER_POS);
            gate.setPosition(GATE_POS);

            telemetry.addData("Turret / Turret1", TURRET_POS);
            telemetry.addData("Hood (L / R)", "%.2f / %.2f", HOOD_POS, 1.0 - HOOD_POS);
            telemetry.addData("Flower (L / R)", "%.2f / %.2f", FLOWER_POS, 1.0 - FLOWER_POS);
            telemetry.addData("Gate", "%.2f", GATE_POS);
            telemetry.update();
        }
    }
}