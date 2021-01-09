package org.firstinspires.ftc.teamcode.lib.datatypes;

import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;

public class UTuple {

    public Adresses a_adr;
    public Instructions a_ins;

    public UTuple b_utp;
    public double b_dbl;
    public double[] b_arr;
    public Boolean b_bln;

    public UTuple (Adresses a, UTuple b) {
        a_adr = a;
        b_utp = b;
    }

    public UTuple (Adresses a, Instructions i, double b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (Instructions i, double b) {
        a_ins = i;
        b_dbl = b;
    }

    public UTuple (Adresses a, Instructions i, double[] b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (Instructions i, double[] b) {
        a_ins = i;
        b_arr = b;
    }

    public UTuple (Adresses a, Instructions i, Boolean b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (Instructions i, Boolean b) {
        a_ins = i;
        b_bln = b;
    }

    public UTuple(Adresses a, Instructions i) {
        a_adr = a;
        b_utp = new UTuple(i);
    }
    public UTuple(Instructions i) {
        a_ins = i;
    }
}
