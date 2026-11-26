package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Diag: Odo Pods", group = "Diagnostics")
public class OdoPodTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "FL");
        DcMotor fr = hardwareMap.get(DcMotor.class, "FR");
        DcMotor bl = hardwareMap.get(DcMotor.class, "BL");
        DcMotor br = hardwareMap.get(DcMotor.class, "BR");

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Pods = ~500 ticks/in | Motores = ~20-30 ticks/in");
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("BL", bl.getCurrentPosition());
            telemetry.addData("BR", br.getCurrentPosition());
            telemetry.update();
        }
    }
}