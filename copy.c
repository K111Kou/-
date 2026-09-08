#include <Servo.h>


// ==================================================
// ピン設定
// ==================================================

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


// ==================================================
// サーボ
// ==================================================

Servo head;


// ==================================================
// モーター速度
// ==================================================

const int L_PWM = 160;
const int R_PWM = 140;


// ==================================================
// 距離
// ==================================================

const float wall_distance = 20.0;
const float goal_distance = 5.0;


// ==================================================
// 旋回時間
// ==================================================

const int TURN_90_TIME  = 90;
const int TURN_180_TIME = 180;


// ==================================================
// IRセンサー
// ==================================================

// 障害物を検知したときLOW
const int IR_ACTIVE = LOW;


// ==================================================
// 状態
// ==================================================

enum CourseState {

  // 状態0
  FIRST_LEFT_BOTTLE,

  // 状態1
  OUTBOUND_CORNER,

  // 状態2
  OUTBOUND_RIGHT_BOTTLE,

  // 状態3
  UTURN_POINT,

  // 状態4
  RETURN_LEFT_BOTTLE,

  // 状態5
  RETURN_CORNER,

  // 状態6
  RETURN_RIGHT_BOTTLE,

  // 状態7
  GOAL
};


CourseState state = FIRST_LEFT_BOTTLE;


// ==================================================
// 誤検知対策用
// ==================================================

// 状態に入った時間
unsigned long stateStartTime = 0;


// ------------------------------------------
// IR関連
// ------------------------------------------

// IRが検知可能な状態になったか
bool irArmed = false;

// HIGHが連続した回数
int irHighCount = 0;

// LOWが連続した回数
int irLowCount = 0;


// 状態変更直後はIRを無視
const unsigned long IR_IGNORE_TIME = 500;

// HIGHが何回続けばセンサーを有効化するか
const int IR_HIGH_REQUIRED = 3;

// LOWが何回続けばボトルと判断するか
const int IR_LOW_REQUIRED = 3;


// ------------------------------------------
// 超音波関連
// ------------------------------------------

int frontCount = 0;

// 何回連続で近ければ壁と判断するか
const int FRONT_REQUIRED = 2;


// ==================================================
// 超音波センサー
// ==================================================

float getDistance() {

  // 初期化
  digitalWrite(HC_TRIG, LOW);
  delayMicroseconds(2);

  // 超音波発射
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG, LOW);

  // 反射時間測定
  unsigned long duration =
      pulseIn(HC_ECHO, HIGH, 30000);


  // 反射がなかった
  if (duration == 0) {

    return 999.0;
  }


  // cmに変換
  float frontDistance =
      duration / 58.0;


  return frontDistance;
}


// ==================================================
// 超音波センサーの角度変更
// ==================================================

float getDistanceAt(int angle) {

  head.write(angle);

  delay(300);

  return getDistance();
}


// ==================================================
// 前進
// ==================================================

void moveForward() {

  // 右モーター
  analogWrite(RMO_EA, R_PWM);

  digitalWrite(RMO_MA1, HIGH);
  digitalWrite(RMO_MA2, LOW);


  // 左モーター
  analogWrite(RMO_EB, L_PWM);

  digitalWrite(RMO_MB1, HIGH);
  digitalWrite(RMO_MB2, LOW);
}


// ==================================================
// 停止
// ==================================================

void stopMotor() {

  analogWrite(RMO_EA, 0);

  digitalWrite(RMO_MA1, LOW);
  digitalWrite(RMO_MA2, LOW);


  analogWrite(RMO_EB, 0);

  digitalWrite(RMO_MB1, LOW);
  digitalWrite(RMO_MB2, LOW);
}


// ==================================================
// 左旋回
// ==================================================

void turnLeft() {

  // 右モーター
  analogWrite(RMO_EA, R_PWM);

  digitalWrite(RMO_MA1, HIGH);
  digitalWrite(RMO_MA2, LOW);


  // 左モーター逆転
  analogWrite(RMO_EB, L_PWM);

  digitalWrite(RMO_MB1, LOW);
  digitalWrite(RMO_MB2, HIGH);
}


// ==================================================
// 右旋回
// ==================================================

void turnRight() {

  // 右モーター逆転
  analogWrite(RMO_EA, R_PWM);

  digitalWrite(RMO_MA1, LOW);
  digitalWrite(RMO_MA2, HIGH);


  // 左モーター
  analogWrite(RMO_EB, L_PWM);

  digitalWrite(RMO_MB1, HIGH);
  digitalWrite(RMO_MB2, LOW);
}


// ==================================================
// 左へ90度
// ==================================================

void turnLeft90() {

  stopMotor();

  delay(200);


  turnLeft();

  delay(TURN_90_TIME);


  stopMotor();

  delay(200);
}


// ==================================================
// 右へ90度
// ==================================================

void turnRight90() {

  stopMotor();

  delay(200);


  turnRight();

  delay(TURN_90_TIME);


  stopMotor();

  delay(200);
}


// ==================================================
// Uターン
// ==================================================

void uTurn() {

  stopMotor();

  delay(200);


  turnRight();

  delay(TURN_180_TIME);


  stopMotor();

  delay(200);
}


// ==================================================
// 右へ回避
// ==================================================

void avoidRight() {

  // 右へ
  turnRight();

  delay(350);


  // 前へ
  moveForward();

  delay(600);


  // 左へ戻す
  turnLeft();

  delay(350);


  stopMotor();

  delay(200);
}


// ==================================================
// 左へ回避
// ==================================================

void avoidLeft() {

  // 左へ
  turnLeft();

  delay(350);


  // 前へ
  moveForward();

  delay(600);


  // 右へ戻す
  turnRight();

  delay(350);


  stopMotor();

  delay(200);
}


// ==================================================
// 状態を変更する関数
// ==================================================

void changeState(CourseState nextState) {

  state = nextState;

  // 状態が変わった時間を記録
  stateStartTime = millis();


  // IR判定リセット
  irArmed = false;

  irHighCount = 0;

  irLowCount = 0;


  // 超音波判定リセット
  frontCount = 0;


  Serial.print("STATE CHANGE -> ");

  Serial.println((int)state);
}


// ==================================================
// ボトル判定
// ==================================================
//
// targetPin
//   → 見たい側のIR
//
// otherPin
//   → 反対側のIR
//
// ==================================================

bool bottleDetected(int targetPin, int otherPin) {

  int targetSensor =
      digitalRead(targetPin);

  int otherSensor =
      digitalRead(otherPin);


  // ------------------------------------------------
  // 状態が変わった直後は無視
  // ------------------------------------------------

  if (millis() - stateStartTime < IR_IGNORE_TIME) {

    return false;
  }


  // ------------------------------------------------
  // まだIRを有効にしていない
  //
  // 一度HIGHになるまで待つ
  // ------------------------------------------------

  if (!irArmed) {


    // 対象センサーが何も検知していない
    if (targetSensor != IR_ACTIVE) {

      irHighCount++;


      // HIGHが数回連続
      if (irHighCount >= IR_HIGH_REQUIRED) {

        irArmed = true;

        irHighCount = 0;

        irLowCount = 0;


        Serial.println("IR ARMED");
      }

    }

    else {

      // LOWになったらカウントリセット
      irHighCount = 0;
    }


    return false;
  }


  // ------------------------------------------------
  // IR有効後
  // ------------------------------------------------


  // 対象側だけLOW
  //
  // 両方LOWなら壁などの可能性があるので無視
  if (
    targetSensor == IR_ACTIVE &&
    otherSensor != IR_ACTIVE
  ) {

    irLowCount++;


    // LOWが数回連続
    if (irLowCount >= IR_LOW_REQUIRED) {

      irLowCount = 0;


      Serial.println("BOTTLE DETECTED");


      return true;
    }
  }

  else {

    irLowCount = 0;
  }


  return false;
}


// ==================================================
// 正面の壁判定
// ==================================================

bool frontDetected(
  float distance,
  float threshold
) {


  // 正常な値かつ指定距離以内
  if (
    distance > 0 &&
    distance <= threshold
  ) {

    frontCount++;


    // 数回連続して近い
    if (frontCount >= FRONT_REQUIRED) {

      frontCount = 0;


      Serial.println("FRONT DETECTED");


      return true;
    }
  }

  else {

    frontCount = 0;
  }


  return false;
}


// ==================================================
// setup
// ==================================================

void setup() {

  Serial.begin(115200);


  // --------------------------------
  // 超音波
  // --------------------------------

  pinMode(HC_TRIG, OUTPUT);

  pinMode(HC_ECHO, INPUT);


  // --------------------------------
  // IR
  // --------------------------------

  pinMode(IRLED_L, INPUT);

  pinMode(IRLED_R, INPUT);


  // --------------------------------
  // モーター
  // --------------------------------

  pinMode(RMO_EA, OUTPUT);

  pinMode(RMO_EB, OUTPUT);


  pinMode(RMO_MA1, OUTPUT);

  pinMode(RMO_MA2, OUTPUT);

  pinMode(RMO_MB1, OUTPUT);

  pinMode(RMO_MB2, OUTPUT);


  // --------------------------------
  // サーボ
  // --------------------------------

  head.attach(
    SERVOPIN,
    500,
    2400
  );


  head.write(90);


  // --------------------------------
  // 初期停止
  // --------------------------------

  stopMotor();

  delay(1000);


  // 初期状態の時間
  stateStartTime = millis();
}


// ==================================================
// loop
// ==================================================

void loop() {


  // ==================================================
  // センサー値取得
  // ==================================================

  float frontDistance =
      getDistance();


  int leftIR =
      digitalRead(IRLED_L);


  int rightIR =
      digitalRead(IRLED_R);


  // ==================================================
  // デバッグ表示
  // ==================================================

  Serial.print("STATE: ");

  Serial.print((int)state);


  Serial.print("  LEFT: ");

  Serial.print(leftIR);


  Serial.print("  RIGHT: ");

  Serial.print(rightIR);


  Serial.print("  FRONT: ");

  Serial.println(frontDistance);


  // ==================================================
  // 状態遷移
  // ==================================================

  switch (state) {


    // ==================================================
    // 状態0
    //
    // 行き：最初の左ボトル
    // ==================================================

    case FIRST_LEFT_BOTTLE:


      moveForward();


      if (
        bottleDetected(
          IRLED_L,
          IRLED_R
        )
      ) {

        stopMotor();


        avoidRight();


        changeState(
          OUTBOUND_CORNER
        );
      }


      break;



    // ==================================================
    // 状態1
    //
    // 行き：コーナー
    // ==================================================

    case OUTBOUND_CORNER:


      moveForward();


      if (
        frontDetected(
          frontDistance,
          wall_distance
        )
      ) {

        stopMotor();


        turnLeft90();


        changeState(
          OUTBOUND_RIGHT_BOTTLE
        );
      }


      break;



    // ==================================================
    // 状態2
    //
    // 行き：右ボトル
    // ==================================================

    case OUTBOUND_RIGHT_BOTTLE:


      moveForward();


      if (
        bottleDetected(
          IRLED_R,
          IRLED_L
        )
      ) {

        stopMotor();


        avoidLeft();


        changeState(
          UTURN_POINT
        );
      }


      break;



    // ==================================================
    // 状態3
    //
    // Uターン地点
    // ==================================================

    case UTURN_POINT:


      moveForward();


      if (
        frontDetected(
          frontDistance,
          wall_distance
        )
      ) {

        stopMotor();


        uTurn();


        changeState(
          RETURN_LEFT_BOTTLE
        );
      }


      break;



    // ==================================================
    // 状態4
    //
    // 帰り：左ボトル
    // ==================================================

    case RETURN_LEFT_BOTTLE:


      moveForward();


      if (
        bottleDetected(
          IRLED_L,
          IRLED_R
        )
      ) {

        stopMotor();


        avoidRight();


        changeState(
          RETURN_CORNER
        );
      }


      break;



    // ==================================================
    // 状態5
    //
    // 帰り：コーナー
    // ==================================================

    case RETURN_CORNER:


      moveForward();


      if (
        frontDetected(
          frontDistance,
          wall_distance
        )
      ) {

        stopMotor();


        turnRight90();


        changeState(
          RETURN_RIGHT_BOTTLE
        );
      }


      break;



    // ==================================================
    // 状態6
    //
    // 帰り：右ボトル
    // ==================================================

    case RETURN_RIGHT_BOTTLE:


      moveForward();


      if (
        bottleDetected(
          IRLED_R,
          IRLED_L
        )
      ) {

        stopMotor();


        avoidLeft();


        changeState(
          GOAL
        );
      }


      break;



    // ==================================================
    // 状態7
    //
    // ゴール
    // ==================================================

    case GOAL:


      moveForward();


      if (
        frontDetected(
          frontDistance,
          goal_distance
        )
      ) {

        stopMotor();


        Serial.println("GOAL");


        while (true) {

          stopMotor();

          delay(100);
        }
      }


      break;
  }


  // センサー判定周期
  delay(20);
}
