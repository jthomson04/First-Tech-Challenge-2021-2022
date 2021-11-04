package org.firstinspires.ftc.teamcode;

public enum DuckPosition {
    LEFT, MIDDLE, RIGHT;

    @Override
    public String toString() {
        return this == DuckPosition.LEFT ? "Left" : this == DuckPosition.MIDDLE ? "Middle" : "Right";
    }
}
