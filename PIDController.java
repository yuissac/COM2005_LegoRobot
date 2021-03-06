import java.util.ArrayList;
import java.time.Clock;
import java.lang.Thread;
public class PIDController{
  private Clock clock = Clock.systemUTC();
  private long base;
  private double kp, kd, ki, ref;
  private ArrayList<Double> errors = new ArrayList<Double>();
  private ArrayList<Long> timing = new ArrayList<Long>();

  public void setTarget(double t){
    ref = t;
  }


  public double run(double s_val){
    double e = gen_e(s_val);
    double c = gen_c(e);
    return c;
  }

  private double gen_e(double s_val){
    double e = ref - s_val;
    errors.add(e);

    timing.add(clock.millis()-base);
    return e;
  }

  private double gen_c(double e){
    double dt = timing.get(errors.size()-1)-timing.get(errors.size()-2);
    double d = (e-errors.get(errors.size()-2))/dt;
    //double i = 0;
    //for (int x=1;x<errors.size();x++){
      //i += errors.get(x)*(timing.get(x)-timing.get(x-1));
    //}
    double i = e*dt;
    return kp*e + kd*d + ki*i;
  }

  public PIDController(double p, double i, double d, double r){
    kp = p;
    kd = d;
    ki = i;
    ref = r;
    errors.add(0.0);
    timing.add((long)0.0);
    base = clock.millis();
  }

  public static void main(String[] args){
    double l = 0;
    double t = 100;
    double s = 0;
    PIDController c = new PIDController(1, 0.5, 0.3, 75);
    for (int i=0;i<2000;i++){
      try{
        Thread.sleep(1);
      } catch(InterruptedException ex){
        Thread.currentThread().interrupt();
      }
      s += c.run(t-l);
      l -= s;
      System.out.println(l);
    }
  }
}
