package org.firstinspires.ftc.teamcode.lib.datatypes;

import org.firstinspires.ftc.teamcode.lib.G;

public class UTuple {

    public G.a a_adr;
    public G.i a_ins;

    public UTuple b_utp;
    public double b_dbl;
    public double[] b_arr;
    public Boolean b_bln;

    public UTuple (G.a a, UTuple b) {
        a_adr = a;
        b_utp = b;
    }

    public UTuple (G.a a, G.i i, double b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (G.i i, double b) {
        a_ins = i;
        b_dbl = b;
    }

    public UTuple (G.a a, G.i i, double[] b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (G.i i, double[] b) {
        a_ins = i;
        b_arr = b;
    }

    public UTuple (G.a a, G.i i, Boolean b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (G.i i, Boolean b) {
        a_ins = i;
        b_bln = b;
    }

    public UTuple(G.a a, G.i i) {
        a_adr = a;
        b_utp = new UTuple(i);
    }
    public UTuple(G.i i) {
        a_ins = i;
    }
}
