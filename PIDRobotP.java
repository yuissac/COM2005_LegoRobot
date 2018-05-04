import ShefRobot.*;
import java.util.ArrayList;
import java.time.Clock;
public class PIDRobotP{
  double speedL = 0;
  double speedR = 0;
  Robot robot;
  Motor leftMotor, rightMotor;
  UltrasonicSensor uSensor;
  UltrasonicSensor uSensorp;
  PIDController pid;
  PIDController pidp;
  double dampener = 1;

  public void setTarget(double t){
    pid.setTarget(t);
  }

  public void printSpeed(){
    System.out.println("Speed: ("+speedL+","+speedR+")");
  }

  public void start(){
    if (speedL >= 0){
      leftMotor.setSpeed((int)(dampener*speedL));
      leftMotor.backward();
    } else {
      leftMotor.setSpeed(-(int)(dampener*speedL));
      leftMotor.forward();
    }
    if (speedR >= 0){
      rightMotor.setSpeed((int)(dampener*speedR));
      rightMotor.backward();
    } else {
      rightMotor.setSpeed(-(int)(dampener*speedR));
      rightMotor.forward();
    }
  }

  public void stop(){
    leftMotor.stop();
    rightMotor.stop();
  }

  public void close(){
    leftMotor.stop();
    rightMotor.stop();
    robot.close();
  }

  public void updateSpeed(){
    double dis = uSensor.getDistance()*100;
    double disp = uSensorp.getDistance()*100;
    double oSpeed = pid.run(dis);

    double turn = pidp.run(disp);





    System.out.println("Distance: "+dis);
    System.out.println("Distance P: "+disp);

    speedL = oSpeed-turn*3;
    speedR = oSpeed+turn*3;

    if (speedL >= 0){
      leftMotor.setSpeed((int)(dampener*speedL));
      leftMotor.backward();
    } else {
      leftMotor.setSpeed(-(int)(dampener*speedL));
      leftMotor.forward();
    }
    if (speedR >= 0){
      rightMotor.setSpeed((int)(dampener*speedR));
      rightMotor.backward();
    } else {
      rightMotor.setSpeed(-(int)(dampener*speedR));
      rightMotor.forward();
    }

  }

  public void sleep(int i){
    robot.sleep(i);
  }

  public PIDRobotP(String code, double kp, double ki, double kd, double kpp, double kip, double kdp, double ref, double refp){
    robot = new Robot();
    leftMotor = robot.getLargeMotor(Motor.Port.A);
    rightMotor = robot.getLargeMotor(Motor.Port.B);
    uSensor = robot.getUltrasonicSensor(Sensor.Port.S1);
    uSensorp = robot.getUltrasonicSensor(Sensor.Port.S2);
    pid = new PIDController(kp, kd, ki, ref);
    pidp = new PIDController(kpp, kdp, kip, refp);
  }






  public static void main(String args[]){
    Clock clock = Clock.systemUTC();
    long base = clock.millis();
    long time = clock.millis()-base;
    PIDRobotP robot = new PIDRobotP("dia-lego-b3", 1, 0, 0, 1, 0, 0, 20.0, 20.0);
    System.out.println("Connected, Starting Robot");
    robot.start();
    System.out.println("Started");
    for (int i=0; i<5000; i++){
      robot.updateSpeed();
      robot.printSpeed();
      //robot.sleep(1);
      time = clock.millis()-base;
      if ((((int)time)/10000)%2==1){
        System.out.println("Flip");
        //robot.setTarget(0.3);
      } else {
        System.out.println("Flop");
        //robot.setTarget(0.5);
      }

    };
    robot.stop();
    System.out.println("Stopped");
    robot.close();
    System.out.println("Closed");
  }
}
