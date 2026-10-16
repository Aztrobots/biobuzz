package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Diag: Odo Pods", group = "Diagnostics")
public class Prueba extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor forward = hardwareMap.get(DcMotor.class, "BR");
        DcMotor strafe  = hardwareMap.get(DcMotor.class, "FL");

        forward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Ambos pods van REVERSE: se niega la lectura cruda para mostrar convención Pedro
            int fwdTicks = -forward.getCurrentPosition();
            int strTicks = -strafe.getCurrentPosition();

            telemetry.addLine("Adelante -> Forward (+) | Izquierda -> Strafe (+)");
            telemetry.addData("Forward (BR)", fwdTicks);
            telemetry.addData("Strafe (FL)", strTicks);
            telemetry.update();
        }
    }
}