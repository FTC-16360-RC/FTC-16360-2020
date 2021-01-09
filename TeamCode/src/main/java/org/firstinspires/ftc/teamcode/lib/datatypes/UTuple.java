package org.firstinspires.ftc.teamcode.lib.datatypes;

public class UTuple {

    public enum A {
        NONE,
        SHOOTER,
        KEYBINDINGS,
        ALIGN_TO_POINT,
        INTAKE;
    }

    public enum I {
        NONE,
        SHOOT_ONE,
        SHOOT_THREE,
        NEXT_TARGET,
        PREVIOUS_TARGET,
        ENABLE_INTAKE,
        DISABLE_INTAKE,
        ENABLE_SHOOTER,
        DISABLE_SHOOTER,
        RESET_ORIENTATION,
        RECIEVE_POSITION,

        ENABLE_INTAKE_DEBUG,
        DISABLE_INTAKE_DEBUG,
        ENABLE_LIFT_DEBUG,
        DISABLE_LIFT_DEBUG,
        LOWER_INTAKE_DEBUG,
        ADJUST_FLAP_DEBUG,
        FEED_RING_DEBUG;
    }

    public A a_adr;
    public I a_ins;

    public UTuple b_utp;
    public double b_dbl;
    public double[] b_arr;
    public Boolean b_bln;

    public UTuple (A a, UTuple b) {
        a_adr = a;
        b_utp = b;
    }

    public UTuple (A a, I i, double b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (I i, double b) {
        a_ins = i;
        b_dbl = b;
    }

    public UTuple (A a, I i, double[] b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (I i, double[] b) {
        a_ins = i;
        b_arr = b;
    }

    public UTuple (A a, I i, Boolean b) {
        a_adr = a;
        b_utp = new UTuple(i, b);
    }
    public UTuple (I i, Boolean b) {
        a_ins = i;
        b_bln = b;
    }

    public UTuple(A a, I i) {
        a_adr = a;
        b_utp = new UTuple(i);
    }
    public UTuple(I i) {
        a_ins = i;
    }
}
