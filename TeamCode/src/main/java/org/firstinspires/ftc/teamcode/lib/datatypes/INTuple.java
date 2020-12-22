package org.firstinspires.ftc.teamcode.lib.datatypes;

public class INTuple {

    private int mode;

    public String a = "";
    public double[] b_array = {};
    private double b_double;
    private Boolean b_bool;

    public INTuple(String instruction, double[] data) {
        mode = 0;
        a = instruction;
        b_array = data;
    }
    public INTuple(String instruction, double data) {
        mode = 1;
        a = instruction;
        b_double = data;
    }
    public INTuple(String instruction, Boolean data) {
        mode = 2;
        a = instruction;
        b_bool = data;
    }
    public INTuple(String instruction) {
        a = instruction;
    }
    public INTuple(int mode) {
        this.mode = mode;
    }
    public INTuple() {
        mode = 0;
    }

    public String get_a() {
        return a;
    }

    public double[] get_b() {
        return b_array;
    }

    //OTHER DATATYPES REEEEEEE
}
