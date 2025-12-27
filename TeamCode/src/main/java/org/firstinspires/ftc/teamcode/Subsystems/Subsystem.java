package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    void update(double dt, Telemetry telemetry);
    void stop();
}
