package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Vector {

    public double x,y,z,w;  //no setters/getters because you can't fucking overload functions is this garbage language
    Globals g = new Globals();

    public Vector(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = 0;
    }

    public Vector(double x,double y) {
        this.x = x;
        this.y = y;
        this.z = 0;
        this.w = 0;
    }
    public Vector(double x) {
        this.x = x;
        this.y = 0;
        this.z = 0;
        this.w = 0;
    }

    public Vector() {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }

    public Vector add(Vector in) {
        Vector out = new Vector();
        out.x = this.x + in.x;
        out.y = this.y + in.y;
        out.z = this.z + in.z;
        out.w = this.w + in.w;
        return out;
    }

    public Vector add(Vector in, Vector in2) {
        Vector out = new Vector();
        out.x = in2.x + in.x;
        out.y = in2.y + in.y;
        out.z = in2.z + in.z;
        out.w = in2.w + in.w;
        return out;
    }

    public Vector subtract(Vector in) {
        Vector out = new Vector();
        out.x = this.x - in.x;
        out.y = this.y - in.y;
        out.z = this.z - in.z;
        out.w = this.w - in.w;
        return out;
    }

    public Vector subtract(Vector in0, Vector in) {
        Vector out = new Vector();
        out.x = in0.x + in.x;
        out.y = in0.y + in.y;
        out.z = in0.z + in.z;
        out.w = in0.w + in.w;
        return out;
    }

    public Vector multiply(Vector in, double in0) {
        Vector out = new Vector();
        out.x = in0 * in.x;
        out.y = in0 * in.y;
        out.z = in0 * in.z;
        out.w = in0 * in.w;
        return out;
    }

    public Vector multiply(double in) {
        Vector out = new Vector();
        out.x = in * this.x;
        out.y = in * this.y;
        out.z = in * this.z;
        out.w = in * this.w;
        return out;
    }

    public double angle() {
        double deg = Math.toDegrees(Math.atan2(this.y, this.x));

        if (Math.abs(deg) > 360) {      //converts angle to between -360, 360
            deg = deg%360;
        }

        if (deg < g.e) {               //converts negative angles to positive ones
            deg = 360-deg;
        }

        return deg;
    }

    public double angle(Vector in) {
        double deg = Math.toDegrees(Math.atan2(in.y, in.x));

        if (Math.abs(deg) > 360) {      //converts angle to between -360, 360
            deg = deg%360;
        }

        if (deg < g.e) {               //converts negative angles to positive ones
            deg = 360-deg;
        }

        return deg;
    }
<<<<<<< HEAD
=======

    public Vector2d toVector2d(Vector in) {
        return new Vector2d(in.x, in.y);
    }

    public Vector toVector(Vector2d in) {
        return new Vector(in.getX(), in.getY());
    }
>>>>>>> TestRC
}
