package frc.lib.controller;

public class PIDGains {
  public double P = 0.0;
  public double I = 0.0;
  public double D = 0.0;

  public PIDGains(double p, double i, double d) {
    this.P = p;
    this.I = i;
    this.D = d;
  }

  public PIDGains() {

  }


}