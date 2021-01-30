package org.firstinspires.ftc.teamcode.lib.datatypes;

public class UTuple {

    String key;
    Double[] valD;
    INTuple valI;
    int mode = 0; //0 = Twople, 1 = INTuple
    INTuple inTuple;
    Twople twople;

    public UTuple (String string, Double[] data) {
        valD = data;
        key = string;
        create();
    }
    public UTuple (String string, INTuple intuple) {
        valI = intuple;
        key = string;
        create();
    }
    public UTuple(int mode) {
        create();
    }

    private void create() {

    }
}
