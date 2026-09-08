#include <Servo.h>

/* 超音波センサー */
#define SERVOPIN  9
#define HC_ECHO   2
#define HC_TRIG   10

/* 赤外線センサー */
#define IRLED_L   4
#define IRLED_R   5

/* モーター */
#define RMO_EA    3
#define RMO_EB    6
#define RMO_MB1   7
#define RMO_MB2   8
#define RMO_MA2   11
#define RMO_MA1   12

Servo head;

/* モーターのPWM */
const int L_PWM = 160;
const int R_PWM = 140;

/* 壁との距離 */
const float WALL_DISTANCE = 20.0;
const float GOAL_DISTANCE = 5.0;

/* 旋回時間 */
const int TURN_90_MS = 350;
const int TURN_180_MS = 700;

/* 赤外線センサーの反応 */
const int IR_ACTIVE = LOW;

/* 状態管理 */
enum CourseState {
  OUTBOUND_HORIZONTAL,
  OUTBOUND_VERTICAL,
  RETURN_VERTICAL,
  RETURN_HORIZONTAL,
  GOAL
};

CourseState state = OUTBOUND_HORIZONTAL;


/* 超音波センサーで距離を求める */
float getDistance() {
  digitalWrite(HC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG, LOW);

  unsigned long duration = pulseIn(HC_ECHO, HIGH, 30000);

  if (duration == 0) {
    return 999.0;
  }

  return duration / 58.0;
}


/* 超音波センサーの角度を変えて距離を求める */
float getDistanceAt(int angle) {
  head.write(angle);
  delay(300);
  return getDistance();
}


/* 前進 */
void moveForward() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, HIGH);
  digitalWrite(RMO_MA2, LOW);

  analogWrite(RMO_EB, L_PWM);
  digitalWrite(RMO_MB1, HIGH);
  digitalWrite(RMO_MB2, LOW);
}


/* 停止 */
void stopMotor() {
  analogWrite(RMO_EA, 0);
  digitalWrite(RMO_MA1, LOW);
  digitalWrite(RMO_MA2, LOW);

  analogWrite(RMO_EB, 0);
  digitalWrite(RMO_MB1, LOW);
  digitalWrite(RMO_MB2, LOW);
}


/* 左折 */
void turnLeft() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, HIGH);
  digitalWrite(RMO_MA2, LOW);

  analogWrite(RMO_EB, L_PWM);
  digitalWrite(RMO_MB1, LOW);
  digitalWrite(RMO_MB2, HIGH);
}


/* 右折 */
void turnRight() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, LOW);
  digitalWrite(RMO_MA2, HIGH);

  analogWrite(RMO_EB, L_PWM);
  digitalWrite(RMO_MB1, HIGH);
  digitalWrite(RMO_MB2, LOW);
}


/* 左へ90度 */
void turnLeft90() {
  stopMotor();
  delay(200);
  turnLeft();
  delay(TURN_90_MS);
  stopMotor();
  delay(200);
}


/* 右へ90度 */
void turnRight90() {
  stopMotor();
  delay(200);
  turnRight();
  delay(TURN_90_MS);
  stopMotor();
  delay(200);
}


/* Uターン */
void uTurn() {
  stopMotor();
  delay(200);
  turnRight();
  delay(TURN_180_MS);
  stopMotor();
  delay(200);
}


/* 右へ避ける */
void avoidRight() {
  stopMotor();
  delay(100);
  turnRight();
  delay(350);
  moveForward();
  delay(600);
  turnLeft();
  delay(350);
  stopMotor();
  delay(150);
}


/* 左へ避ける */
void avoidLeft() {
  stopMotor();
  delay(100);
  turnLeft();
  delay(350);
  moveForward();
  delay(600);
  turnRight();
  delay(350);
  stopMotor();
  delay(150);
}


/* 左側の赤外線センサーを確認 */
bool leftObstacle() {
  if (digitalRead(IRLED_L) == IR_ACTIVE) {
    delay(20);

    if (digitalRead(IRLED_L) == IR_ACTIVE) {
      return true;
    }
  }

  return false;
}


/* 右側の赤外線センサーを確認 */
bool rightObstacle() {
  if (digitalRead(IRLED_R) == IR_ACTIVE) {
    delay(20);

    if (digitalRead(IRLED_R) == IR_ACTIVE) {
      return true;
    }
  }

  return false;
}


/* 壁を確認 */
bool checkWall(float distance, float threshold) {
  if (distance > 0 && distance <= threshold) {
    delay(20);

    float distance2 = getDistance();

    if (distance2 > 0 && distance2 <= threshold) {
      return true;
    }
  }

  return false;
}


void setup() {
  Serial.begin(115200);

  /* 超音波センサー */
  pinMode(HC_TRIG, OUTPUT);
  pinMode(HC_ECHO, INPUT);

  /* 赤外線センサー */
  pinMode(IRLED_L, INPUT);
  pinMode(IRLED_R, INPUT);

  /* モーター */
  pinMode(RMO_EA, OUTPUT);
  pinMode(RMO_EB, OUTPUT);
  pinMode(RMO_MA1, OUTPUT);
  pinMode(RMO_MA2, OUTPUT);
  pinMode(RMO_MB1, OUTPUT);
  pinMode(RMO_MB2, OUTPUT);

  /* サーボモーター */
  head.attach(SERVOPIN, 500, 2400);
  head.write(90);

  stopMotor();
  delay(1000);
}


void loop() {
  float frontDistance = getDistance();

  /* センサーの値を表示 */
  Serial.print("状態: ");
  Serial.println(state);
  Serial.print("左IR: ");
  Serial.println(digitalRead(IRLED_L));
  Serial.print("右IR: ");
  Serial.println(digitalRead(IRLED_R));
  Serial.print("正面: ");
  Serial.println(frontDistance);

  switch (state) {

    /* スタートから横方向 */
    case OUTBOUND_HORIZONTAL:
      if (checkWall(frontDistance, WALL_DISTANCE)) {
        stopMotor();
        turnLeft90();
        state = OUTBOUND_VERTICAL;
        break;
      }

      if (leftObstacle()) {
        stopMotor();
        avoidRight();
        break;
      }

      moveForward();
      break;


    /* 往路の縦方向 */
    case OUTBOUND_VERTICAL:
      if (checkWall(frontDistance, WALL_DISTANCE)) {
        stopMotor();
        uTurn();
        state = RETURN_VERTICAL;
        break;
      }

      if (rightObstacle()) {
        stopMotor();
        avoidLeft();
        break;
      }

      moveForward();
      break;


    /* 復路の縦方向 */
    case RETURN_VERTICAL:
      if (checkWall(frontDistance, WALL_DISTANCE)) {
        stopMotor();
        turnRight90();
        state = RETURN_HORIZONTAL;
        break;
      }

      if (leftObstacle()) {
        stopMotor();
        avoidRight();
        break;
      }

      moveForward();
      break;


    /* 復路の横方向 */
    case RETURN_HORIZONTAL:
      if (checkWall(frontDistance, GOAL_DISTANCE)) {
        stopMotor();
        state = GOAL;
        break;
      }

      if (rightObstacle()) {
        stopMotor();
        avoidLeft();
        break;
      }

      moveForward();
      break;


    /* ゴール */
    case GOAL:
      stopMotor();
      break;
  }

  delay(20);
}
