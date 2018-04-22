import ShefRobot.*;
public class PIDRobot{
  double speed = 0;
  Robot robot;
  Motor leftMotor, rightMotor;
  UltrasonicSensor uSensor;
  PIDController pid;

  public void start(){
    if (speed >= 0){
      leftMotor.setSpeed((int)speed);
      rightMotor.setSpeed((int)speed);
      leftMotor.forward();
      rightMotor.forward();
    } else {
      leftMotor.setSpeed(-(int)speed);
      rightMotor.setSpeed(-(int)speed);
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
    speed += pid.run(uSensor.getDistance());
    if (speed*oSpeed < 0){
      leftMotor.stop();
      rightMotor.stop();
      if (speed >= 0){
        leftMotor.setSpeed((int)speed);
        rightMotor.setSpeed((int)speed);
        leftMotor.forward();
        rightMotor.forward();
      } else {
        leftMotor.setSpeed(-(int)speed);
        rightMotor.setSpeed(-(int)speed);
        leftMotor.backward();
        rightMotor.backward();
      }
    } else {
      if (speed >= 0){
        leftMotor.setSpeed((int)speed);
        rightMotor.setSpeed((int)speed);
      } else {
        leftMotor.setSpeed(-(int)speed);
        rightMotor.setSpeed(-(int)speed);
      }
    }
  }

  public PIDRobot(String code, double kp, double kd, double ki, double ref){
    robot = new Robot(code);
    leftMotor = robot.getLargeMotor(Motor.Port.A);
    rightMotor = robot.getLargeMotor(Motor.Port.B);
    uSensor = robot.getUltrasonicSensor(Sensor.Port.S1);
    pid = new PIDController(kp, kd, ki, ref);
  }






  public static void main(String args[]){
    PIDRobot robot = new PIDRobot("dia-lego-b3", 1, 0.5, 0.3, 20);
    robot.start();
    for (int i=0; i<200; i++){
      robot.updateSpeed();
    };
    robot.stop();
    robot.close();
  }
}
