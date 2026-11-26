package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

/**
 * Pose al final del autónomo para que el TeleOp arranque donde terminó.
 * Es static: sobrevive entre OpModes mientras no se reinicie la app del Robot Controller.
 */
public class PoseStorage {
    public static double MAX_JUMP_IN = 6.0; // ningún robot se mueve 6 in en un loop: si pasa, la lectura es basura

    public static Pose pose = null; // null = no hay pose guardada, el TeleOp usa su START

    /**
     * Guarda la pose solo si es una lectura real. Al detener un OpMode el Pinpoint puede
     * regresar (0, 0, 0) en el último loop; sin este filtro esa basura pisaba la pose buena.
     */
    public static void save(Pose p) {
        if (p == null || Double.isNaN(p.x()) || Double.isNaN(p.y()) || Double.isNaN(p.heading())) return;
        if (pose != null && Math.hypot(p.x() - pose.x(), p.y() - pose.y()) > MAX_JUMP_IN) return;
        pose = p;
    }
}
