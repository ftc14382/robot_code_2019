package org.firstinspires.ftc.teamcode;

public class Planner {
    double xStart;
    double yIntercept = 0.1;
    double yMax = 1;
    double target;
    double a = 0.05;
    double b = -0.5;
    double x1;
    double x2;

    public Planner(double startX, double t, double power) {
        xStart = startX;
        target = t;
        yMax = power;
        x1 = startX + (yMax-yIntercept)/a;
        x2 = yMax/b+target;
        if(x1>x2) {
            x1 = (-b * target - yIntercept) / (a - b);
            x2 = x1;
        }
        x1 = Math.abs(x1);
        x2 = Math.abs(x2);
    }

    public double getPower(double x) {
        x= Math.abs(x);
        if(x<x1){
            return(x*a+yIntercept);
        } else if(x>x2) {
            return (b*(x-target));
        } else {
            return (yMax);
        }
    }
}
