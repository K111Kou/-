#include <Servo.h>
/*超音波センサー*/
#define SERVOPIN  9
#define HC_ECHO   2   //  OUT 超音波
#define HC_TRIG  10   //  OUT 超音波

/*赤外線センサー  */
#define IRLED_L    4   //  INP 赤外 L
#define IRLED_R    5   //  INP 赤外 R

/*モーター*/
#define RMO_EA    3   //  OUT ENA モータPWM
#define RMO_EB    6   //  OUT ENB モータPWM
#define RMO_MB1   7   //  OUT IN3 モータ左
#define RMO_MB2   8   //  OUT IN4 モータ左
#define RMO_MA2  11   //  OUT IN2 モータ右
#define RMO_MA1  12   //  OUT IN1 モータ右

boolean R_IRLED_R;    //　赤外右
boolean R_IRLED_L;    //　赤外左


#include <Servo.h>
#define SERVOPIN  9
Servo head;          // Servoオブジェクトの宣言

/*左右モーターのPWM*/
const int L_PWM = 160;
const int R_PWM = 140;

/*壁との距離*/
const float wall_distance =20;
const float goal_distance =5;

/*サーボモーターの角度*/
const int TURN_0_TIME = 0;
const int TURN_90_TIME = 90;
const int TURN_180_TIME = 180;

/*状態管理用*/
enum CourceState {
  FIRST_LEFT_BOTTELE,
  OUTBOUND_CORNER,
  OUTBOUND_RIGHT_BOTTELE,
  UTURN_POINT,
  RETURN_LEFT_BOTTELE,
  RETURN_CORNER,
  RETURN_RIGHT_BOTTELE,
  GOAL
};

CourceState state = FIRST_LEFT_BOTTELE; 
int IR_ACTIVE = LOW;


/*超音波センサーで距離を求める関数*/
float getDistance() {
  digitalWrite(HC_TRIG, LOW); //両方のピンをLOWに指定して初期化
  delayMicroseconds(2);
  digitalWrite(HC_TRIG, HIGH); //超音波パルスを発射
  delayMicroseconds(10);      //10us続ける
  digitalWrite(HC_TRIG, LOW); //発射停止
  unsigned long duration = pulseIn(HC_ECHO, HIGH, 30000);

  if (duration == 0) {
    return 999.0;
  }

    float frontDistance = duration / 58.0 ; //距離をHScmに格納

    return frontDistance;
}

/*超音波センサーの角度を変え距離を求める関数*/
float getDistanceAt(int angle) {
  head.write(angle);
  delay(300);
  return getDistance();
}


//前進
void moveForward() {
  analogWrite(RMO_EA, R_PWM);       
  digitalWrite(RMO_MA1, HIGH );
  digitalWrite(RMO_MA2, LOW );
  analogWrite(RMO_EB, L_PWM);       
  digitalWrite(RMO_MB1, HIGH );
  digitalWrite(RMO_MB2, LOW);
}

//停止
void stopMortor() {
  analogWrite(RMO_EA, 0);       
  digitalWrite(RMO_MA1, LOW );
  digitalWrite(RMO_MA2, LOW );
  analogWrite(RMO_EB, 0);       
  digitalWrite(RMO_MB1, LOW );
  digitalWrite(RMO_MB2, LOW);
}

//左折
void turnLeft() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, HIGH );
  digitalWrite(RMO_MA2, LOW );
  analogWrite(RMO_EB, L_PWM);      
  digitalWrite(RMO_MB1, LOW );
  digitalWrite(RMO_MB2, HIGH );
}

//右折
void turnRight() {
  analogWrite(RMO_EA, R_PWM);
  digitalWrite(RMO_MA1, LOW );
  digitalWrite(RMO_MA2, HIGH );
  analogWrite(RMO_EB, L_PWM);      
  digitalWrite(RMO_MB1, HIGH );
  digitalWrite(RMO_MB2, LOW );
}

//回避動作
//左へ90°
void turnLeft90() {
  stopMortor();
  delay(200);
  turnLeft();
  delay(TURN_90_TIME);
  stopMortor();
  delay(200);
}

//右へ90°
void turnRight90() {
  stopMortor();
  delay(200);
  turnRight();
  delay(TURN_90_TIME);
  stopMortor();
  delay(200);
}

//Uターン
void uTurn() {
  stopMortor();
  delay(200);
  turnRight();
  delay(TURN_180_TIME);
  stopMortor();
  delay(200);
}

//右へ避ける
void avoidRight() {
  turnRight();
  delay(350);
  moveForward();
  delay(600);
  turnLeft();
  delay(350);
  stopMortor();
  delay(200);
}

//左へ避ける
void avoidLeft() {
  turnLeft();
  delay(350);
  moveForward();
  delay(600);
  turnRight();
  delay(350);
  stopMortor();
  delay(200);
}




void setup() {
  Serial.begin(115200 );
  /*超音波センサー*/
  pinMode(HC_TRIG, OUTPUT);
  pinMode(HC_ECHO, INPUT);

  /*赤外線センサー*/
  pinMode(IRLED_L, INPUT);
  pinMode(IRLED_R, INPUT);
  
  /*4輪モーター*/
  pinMode(RMO_EA, OUTPUT);
  pinMode(RMO_EB, OUTPUT);

  /*モーターのPWM*/
  pinMode(RMO_MA1, OUTPUT);
  pinMode(RMO_MA2, OUTPUT);
  pinMode(RMO_MB1, OUTPUT);
  pinMode(RMO_MB2, OUTPUT);

  head.attach(SERVOPIN,500,2400); //サーボモーターの設定
  head.write(90); //サーボモーターを90度にする

  stopMortor();
  delay(1000);

}

void loop() {
  float frontDistance = getDistance();
  int R_IRLED_L = digitalRead(IRLED_L);
  int R_IRLED_R = digitalRead(IRLED_R);


  Serial.print("状態; ");
  Serial.println(state);
  Serial.print("左IR; ");
  Serial.println(R_IRLED_L);
  Serial.print("右IR; ");
  Serial.println(R_IRLED_R);
  Serial.print("正面; ");
  Serial.println(frontDistance);
  switch(state) {
    case FIRST_LEFT_BOTTELE:
      moveForward();
      if (R_IRLED_L == LOW) {
        stopMortor();
        avoidRight();
        state = OUTBOUND_CORNER;      
      } 
      break;
  

    case OUTBOUND_CORNER:
      moveForward();
      if (frontDistance > 0 && frontDistance <= wall_distance) {
        turnLeft90();
        state = OUTBOUND_RIGHT_BOTTELE;
      } 
      break;
    
    case OUTBOUND_RIGHT_BOTTELE:
      moveForward();
      if (R_IRLED_R == LOW) {
        stopMortor();
        avoidLeft();
        state = UTURN_POINT;
      }
      break;

    case UTURN_POINT:
      moveForward();
      if (frontDistance > 0 && frontDistance<= wall_distance) {
        uTurn();
        state = RETURN_LEFT_BOTTELE;
      }
      break;
    
    case RETURN_LEFT_BOTTELE:
      moveForward();
      if (R_IRLED_L == LOW) {
        stopMortor();
        avoidRight();
        state = RETURN_CORNER;      
      } 
      break;

    case RETURN_CORNER:
      moveForward();
      if (frontDistance > 0 && frontDistance <= wall_distance) {
        turnRight90();
        state = RETURN_RIGHT_BOTTELE;
      } 
      break;
  
    case RETURN_RIGHT_BOTTELE:
      moveForward();
      if (R_IRLED_R == LOW) {
        stopMortor();
        avoidLeft();
        state = GOAL;
      }
      break;
  
    case GOAL:
      moveForward();
      if (frontDistance > 0 && frontDistance <= goal_distance) {
        stopMortor();

        while(true) {
          stopMortor();
        }
      }
      break;
  }
  delay(20);
}
