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
/* モーター設定 */
const int L_PWM = 160;
const int R_PWM = 140;


/* 距離設定 */
/* 正面との距離 */
const float WALL_DISTANCE = 20.0;

/* ゴールの停止距離 */
const float GOAL_DISTANCE = 5.0;

/* 旋回時間 */
const int TURN_90_MS  = 350;
const int TURN_180_MS = 700;

/* LOWのとき検知するセンサーとして扱う */
const int IR_ACTIVE = LOW;

/*
   一瞬の誤検知を無視するため、
   連続3回反応したら障害物と判断
*/
const int IR_CONFIRM_COUNT = 3;

int leftIRCount  = 0;
int rightIRCount = 0;


/*
   回避終了直後に同じボトルをもう一度
   検知しないための無視時間
*/
const unsigned long IR_COOLDOWN = 800;

unsigned long ignoreIRUntil = 0;


/* =========================
   コース状態
   ========================= */

enum CourseState {
  OUTBOUND_HORIZONTAL,
  OUTBOUND_VERTICAL,
  RETURN_VERTICAL,
  RETURN_HORIZONTAL,
  GOAL
};

CourseState state = OUTBOUND_HORIZONTAL;


/* =========================
   壁判定用
   ========================= */

/*
   超音波も1回だけでは誤測定する可能性があるため
   連続して壁が見えた場合だけ曲がる
*/

const int WALL_CONFIRM_COUNT = 3;

int wallCount = 0;


/* =========================
   超音波
   ========================= */

float getDistance() {
  digitalWrite(HC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG, LOW);
  unsigned long duration =
      pulseIn(HC_ECHO, HIGH, 30000);
  if (duration == 0) {
    return 999.0;
  }
  return duration / 58.0;
}


/* サーボ角度を変えて測定 */
float getDistanceAt(int angle) {
  head.write(angle);
  delay(300);
  return getDistance();
}


/* =========================
   モーター制御
   ========================= */

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


/* 左旋回 */
void turnLeft() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, HIGH);
  digitalWrite(RMO_MA2, LOW);
  analogWrite(RMO_EB, L_PWM);
  digitalWrite(RMO_MB1, LOW);
  digitalWrite(RMO_MB2, HIGH);
}


/* 右旋回 */
void turnRight() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, LOW);
  digitalWrite(RMO_MA2, HIGH);
  analogWrite(RMO_EB, L_PWM);
  digitalWrite(RMO_MB1, HIGH);
  digitalWrite(RMO_MB2, LOW);
}


/* =========================
   90度旋回
   ========================= */

void turnLeft90() {
  stopMotor();
  delay(200);
  turnLeft();
  delay(TURN_90_MS);
  stopMotor();
  delay(200);
}


void turnRight90() {
  stopMotor();
  delay(200);
  turnRight();
  delay(TURN_90_MS);
  stopMotor();
  delay(200);
}


/* =========================
   Uターン
   ========================= */

void uTurn() {
  stopMotor();
  delay(200);
  turnRight();
  delay(TURN_180_MS);
  stopMotor();
  delay(200);
}


/* =========================
   ボトル回避
   ========================= */

/* 左側障害物を回避 */
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

/* 右側障害物を回避 */
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


/* =========================
   IRセンサー更新
   ========================= */

void updateIRSensors() {
  int leftRaw  = digitalRead(IRLED_L);
  int rightRaw = digitalRead(IRLED_R);
  /* 左 */
  if (leftRaw == IR_ACTIVE) {
    if (leftIRCount < IR_CONFIRM_COUNT) {
      leftIRCount++;
    }
  } else {
    leftIRCount = 0;
  }

  /* 右 */
  if (rightRaw == IR_ACTIVE) {
    if (rightIRCount < IR_CONFIRM_COUNT) {
      rightIRCount++;
    }
  } else {
    rightIRCount = 0;
  }
}


/* 左側の障害物 */
bool leftObstacle() {
  return leftIRCount >= IR_CONFIRM_COUNT;
}

/* 右側の障害物 */
bool rightObstacle() {
  return rightIRCount >= IR_CONFIRM_COUNT;
}

/* IRカウンタをリセット */
void resetIR() {
  leftIRCount = 0;
  rightIRCount = 0;
}


/* =========================
   壁判定
   ========================= */

bool checkWall(float distance, float threshold) {
  if (distance > 0 &&
      distance <= threshold) {
    wallCount++;
  } else {
    wallCount = 0;
  }

  if (wallCount >= WALL_CONFIRM_COUNT) {
    wallCount = 0;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  /* 超音波 */
  pinMode(HC_TRIG, OUTPUT);
  pinMode(HC_ECHO, INPUT);
  /* IR */
  pinMode(IRLED_L, INPUT);
  pinMode(IRLED_R, INPUT);
  /* モーター */
  pinMode(RMO_EA, OUTPUT);
  pinMode(RMO_EB, OUTPUT);
  pinMode(RMO_MA1, OUTPUT);
  pinMode(RMO_MA2, OUTPUT);
  pinMode(RMO_MB1, OUTPUT);
  pinMode(RMO_MB2, OUTPUT);

  /* サーボ */
  head.attach(SERVOPIN, 500, 2400);
  /* 正面 */
  head.write(90);
  stopMotor();
  delay(1000);
}


void loop() {
  float frontDistance = getDistance();
  updateIRSensors();
  /* デバッグ表示 */
  Serial.print("STATE: ");
  Serial.println(state);
  Serial.print("LEFT IR: ");
  Serial.println(leftObstacle());
  Serial.print("RIGHT IR: ");
  Serial.println(rightObstacle());
  Serial.print("FRONT: ");
  Serial.println(frontDistance);
  Serial.println("----------------");

  bool canUseIR =
      millis() >= ignoreIRUntil;
  switch (state) {
    /* ===================================
       往路：横方向
       =================================== */
    case OUTBOUND_HORIZONTAL:
      if (checkWall(
            frontDistance,
            WALL_DISTANCE)) {
        stopMotor();
        turnLeft90();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        state = OUTBOUND_VERTICAL;
        break;
      }

      if (canUseIR &&
          leftObstacle()) {
        stopMotor();
        avoidRight();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        break;
      }
      moveForward();
      break;



    /* ===================================
       往路：縦方向
       =================================== */
    case OUTBOUND_VERTICAL:
      if (checkWall(
            frontDistance,
            WALL_DISTANCE)) {
        stopMotor();
        uTurn();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        state = RETURN_VERTICAL;
        break;
      }

      if (canUseIR &&
          rightObstacle()) {
        stopMotor();
        avoidLeft();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        break;
      }
      moveForward();
      break;



    /* ===================================
       復路：縦方向
       =================================== */
    case RETURN_VERTICAL:
      if (checkWall(
            frontDistance,
            WALL_DISTANCE)) {
        stopMotor();
        turnRight90();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        state = RETURN_HORIZONTAL;
        break;
      }

      if (canUseIR &&
          leftObstacle()) {
        stopMotor();
        avoidRight();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        break;
      }

      moveForward();
      break;

    /* ===================================
       復路：横方向
       =================================== */

    case RETURN_HORIZONTAL:
      if (checkWall(
            frontDistance,
            GOAL_DISTANCE)) {
        stopMotor();
        state = GOAL;
        break;
      }

      if (canUseIR &&
          rightObstacle()) {
        stopMotor();
        avoidLeft();
        resetIR();
        ignoreIRUntil =
            millis() + IR_COOLDOWN;
        break;
      }
      moveForward();
      break;

    /* ===================================
       ゴール
       =================================== */

    case GOAL:
      stopMotor();
      break;
  }
  delay(20);
}
