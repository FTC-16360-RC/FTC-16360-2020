package org.firstinspires.ftc.teamcode.lib.datatypes;

public class Twople {

    public String a;
    public INTuple b;

    public String get_a() {
        return a;
    }
    public INTuple get_b() {
        return b;
    }
    public Twople (String Recipiant, INTuple Instructions) {
        a = Recipiant;
        b = Instructions;
    }
    public Twople () {
        a = "";
        b = new INTuple(0);
    }

}
