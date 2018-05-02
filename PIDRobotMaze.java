import ShefRobot.*;
import java.util.ArrayList;
import java.time.Clock;
public class PIDRobotMaze{
  private double speedL = 0;
  private double speedR = 0;
  private Robot robot;
  private Motor leftMotor, rightMotor;
  private UltrasonicSensor uSensorL;
  private UltrasonicSensor uSensorR;
  private TouchSensor qSensor;
  private PIDController pidL;
  private PIDController pidR;
  private double pDisL = 0;
  private double pDisR = 0;
  private double dampener = 0.12;

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
    double disL = uSensorL.getDistance()*100;
    double disR = uSensorR.getDistance()*100;

    if (disL >250){
      disL = pDisL;
    };
    if (disR >250){
      disR = pDisR;
    };
    pDisL = disL;
    pDisR = disR;

    double sL = pidL.run(disL);
    double sR = pidR.run(disR);

    System.out.println("PID L: "+sL);
    System.out.println("PID R: "+sR);

    System.out.println("Distance L: "+disL);
    System.out.println("Distance R: "+disR);

    speedL = sR*0.3+sL*1;
    speedR = sR*1+sL*0.3;

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

  public boolean checkQuit(){
    return qSensor.isTouched();
  }

  public void sleep(int i){
    robot.sleep(i);
  }

  public PIDRobotMaze(String code, double kpL, double kiL, double kdL, double kpR, double kiR, double kdR, double refL, double refR){
    robot = new Robot();
    leftMotor = robot.getLargeMotor(Motor.Port.A);
    rightMotor = robot.getLargeMotor(Motor.Port.B);
    uSensorL = robot.getUltrasonicSensor(Sensor.Port.S1);
    uSensorR = robot.getUltrasonicSensor(Sensor.Port.S2);
    qSensor = robot.getTouchSensor(Sensor.Port.S3);
    pidL = new PIDController(kpL, kdL, kiL, refL);
    pidR = new PIDController(kpR, kdR, kiR, refR);
  }


  public static void main(String args[]){
    Clock clock = Clock.systemUTC();
    long base = clock.millis();
    long time = clock.millis()-base;
    PIDRobotMaze robot = new PIDRobotMaze("dia-lego-b3", 20, 0.1*0.375, 0.1*0.09375, 20, 0.1*0.375, 0.1*0.09375, 1.0, 1.0);

    System.out.println("Connected, Starting Robot");
    robot.start();
    System.out.println("Started");

    while(true){
      robot.updateSpeed();
      robot.printSpeed();
      robot.sleep(20);
      if (robot.checkQuit()){
        break;
      }
    };

    robot.stop();
    System.out.println("Stopped");
    robot.close();
    System.out.println("Closed");
  }
}
