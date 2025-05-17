package org.firstinspires.ftc.teamcode.robotarm;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public interface Path {
    VectorF getPoint (double t);
}
