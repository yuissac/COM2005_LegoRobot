import java.util.ArrayList;
public class PIDController{
  private double kp, kd, ki, ref;
  private ArrayList<Double> errors = new ArrayList<Double>();


  public double run(double s_val){
    double e = gen_e(s_val);
    double c = gen_c(e);
    return c;
  }

  private double gen_e(double s_val){
    double e = ref - s_val;
    errors.add(e);
    return e;
  }

  private double gen_c(double e){
    double d = e-errors.get(errors.size()-2);
    double i = 0;
    for (Double x: errors){
      i += x;
    };
    return kp*e + kd*d + ki*i;
  }

  public PIDController(double p, double d, double i, double r){
    kp = p;
    kd = d;
    ki = i;
    ref = r;
    errors.add(0.0);
  }

  public static void main(String[] args){
    double l = 0;
    double t = 400;
    double s = 0;
    PIDController c = new PIDController(1, 0.5, 0.3, 50);
    for (int i=0;i<100;i++){
      s += c.run(t-l);
      l -= s;
      System.out.println(l);
    }
  }
}
