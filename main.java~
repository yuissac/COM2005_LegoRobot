import ShefRobot.*;
public class main{
  public static void main(String args[]){
    Robot robot = new Robot("dia-lego-b3");

    Motor leftMotor = robot.getLargeMotor(Motor.Port.A);
    Motor rightMotor = robot.getLargeMotor(Motor.Port.B);
    UltrasonicSensor ultraSensor = robot.getUltrasonicSensor(Sensor.Port.S1);

    SensorThread p = new SensorThread(ultraSensor);
    p.start();

    leftMotor.setSpeed(300);
    rightMotor.setSpeed(300);
    leftMotor.forward();
    rightMotor.forward();

    robot.sleep(1000);

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    leftMotor.stop();
    rightMotor.stop();


    p.stop();
    robot.close();
  }
}
class SensorThread extends Thread {
       UltrasonicSensor ultraSensor;
       SensorThread(UltrasonicSensor u) {
         ultraSensor = u;
       }

       public void run() {
         while(true){
           System.out.println("Distance "+ultraSensor.getDistance());
         }

       }
   }
