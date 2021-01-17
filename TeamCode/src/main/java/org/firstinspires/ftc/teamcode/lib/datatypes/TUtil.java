package org.firstinspires.ftc.teamcode.lib.datatypes;

import org.firstinspires.ftc.teamcode.lib.datatypes.util.Adresses;
import org.firstinspires.ftc.teamcode.lib.datatypes.util.Instructions;

import java.util.ArrayList;

public class TUtil {

    public ArrayList<UTuple> list = new ArrayList<>();
    public int size = 0;

    public TUtil () {
    }
    public TUtil (int i) {
    }

    private void updateSize() {
        size = list.size();
    }
    public void add(UTuple in) {
        list.add(in);
        updateSize();
    }
    public void add(Adresses a, Instructions i) {
        list.add(new UTuple(a, i));
        updateSize();
    }
    public void add(Adresses a, Instructions i, double d) {
        list.add(new UTuple(a, i, d));
        updateSize();
    }
    public void add(Adresses a, Instructions i, double[] d) {
        list.add(new UTuple(a, i, d));
        updateSize();
    }
    public void add(Adresses a, Instructions i, Boolean b) {
        list.add(new UTuple(a, i, b));
        updateSize();
    }
    public ArrayList<UTuple> foreach() {
        return list;
    }
    public int size() {
        return list.size();
    }
    public UTuple get(int i) {
        return list.get(i);
    }
    public void clear() {
        list.clear();
    }
}
