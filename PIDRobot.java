import ShefRobot.*;
import java.util.ArrayList;
import java.time.Clock;
public class PIDRobot{
  double speed = 0;
  Robot robot;
  Motor leftMotor, rightMotor;
  UltrasonicSensor uSensor;
  PIDController pid;
  double dampener = 1;

  public void setTarget(double t){
    pid.setTarget(t);
  }

  public void printSpeed(){
    System.out.println("Speed: "+speed);
  }

  public void start(){
    if (speed >= 0){
      leftMotor.setSpeed((int)(dampener*speed));
      rightMotor.setSpeed((int)(dampener*speed));
      leftMotor.forward();
      rightMotor.forward();
    } else {
      leftMotor.setSpeed(-(int)(dampener*speed));
      rightMotor.setSpeed(-(int)(dampener*speed));
      leftMotor.backward();
      rightMotor.backward();
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
    double oSpeed = speed;
    double dis = uSensor.getDistance()*100;
    speed = pid.run(dis);
    System.out.println("Distance: "+dis);

    if (speed >= 0){
      leftMotor.setSpeed((int)(dampener*speed));
      rightMotor.setSpeed((int)(dampener*speed));
      leftMotor.backward();
      rightMotor.backward();
    } else {
      leftMotor.setSpeed(-(int)(dampener*speed));
      rightMotor.setSpeed(-(int)(dampener*speed));
      leftMotor.forward();
      rightMotor.forward();
    }

  }

  public void sleep(int i){
    robot.sleep(i);
  }

  public PIDRobot(String code, double kp, double kd, double ki, double ref){
    robot = new Robot(code);
    leftMotor = robot.getLargeMotor(Motor.Port.A);
    rightMotor = robot.getLargeMotor(Motor.Port.B);
    uSensor = robot.getUltrasonicSensor(Sensor.Port.S1);
    pid = new PIDController(kp, kd, ki, ref);
  }






  public static void main(String args[]){
    Clock clock = Clock.systemUTC();
    long base = clock.millis();
    long time = clock.millis()-base;
    PIDRobot robot = new PIDRobot("dia-lego-b3", 3, 0.28125, 8, 50);
    System.out.println("Connected, Starting Robot");
    robot.start();
    System.out.println("Started");
    for (int i=0; i<5000; i++){
      robot.updateSpeed();
      robot.printSpeed();
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
