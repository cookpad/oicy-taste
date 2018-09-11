/*
    OiCy Taste Prototype Firmware

*/

#define VERSION "taste_beta2_20180910_os"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <MCP23S17.h>
#include <SPI.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <ESP8266HTTPClient.h>
#include <Ticker.h>

#define SYSTEM_CLOCK  80000000L
#define BAUDRATE 115200

#define TIMER_1SEC        SYSTEM_CLOCK                // 80MHz == 1sec
#define TIMER_100MSEC     ( TIMER_1SEC / 10 )
#define TIMER_10MSEC      ( TIMER_1SEC / 100 )
#define TIMER_1MSEC       ( TIMER_1SEC / 1000 )

#define TIMER_INTERVAL    TIMER_10MSEC
#define SELECTLED_TOGGLE_INTERVAL 500 //msec

#define NUM_OF_BOTTLES 4

// ---------------------------------------------------------
// ピン番号の指定
// 必要に応じてここを変える
// ---------------------------------------------------------
#define   SOLENOID          16                              //@ 電磁弁
#define   BOOTMODE_BUTTON   0
#define   SPI_CS_PORT_NUM   15                              //@ I/O 15 エキスパンダのSPI用CS

#define   EXGPIO_GPA0       0
#define   EXGPIO_GPA1       1
#define   EXGPIO_GPA2       2
#define   EXGPIO_GPA3       3
#define   EXGPIO_GPA4       4
#define   EXGPIO_GPA5       5
#define   EXGPIO_GPA6       6
#define   EXGPIO_GPA7       7
#define   EXGPIO_GPB0       8
#define   EXGPIO_GPB1       9
#define   EXGPIO_GPB2       10
#define   EXGPIO_GPB3       11
#define   EXGPIO_GPB4       12
#define   EXGPIO_GPB5       13
#define   EXGPIO_GPB6       14
#define   EXGPIO_GPB7       15

#define   KEY_RECIPE_DISCHARGE  EXGPIO_GPA0
#define   KEY_SOY_SAUCE_SEL     EXGPIO_GPA1
#define   KEY_MIRIN_SEL         EXGPIO_GPA2
#define   KEY_SAKE_SEL          EXGPIO_GPA3
#define   KEY_VINEGAR_SEL       EXGPIO_GPA4
#define   KEY_ROTARY_A          EXGPIO_GPA5
#define   KEY_ROTARY_B          EXGPIO_GPA6
#define   KEY_ROTARY_PUSH       EXGPIO_GPA7
#define   LED_SOY_SAUCE         EXGPIO_GPB0
#define   LED_MIRIN             EXGPIO_GPB1
#define   LED_SAKE              EXGPIO_GPB2
#define   LED_VINEGAR           EXGPIO_GPB3

#define MASK_ROOFDISPENSE B00000001
#define MASK_SOYSOURCE    B00000010
#define MASK_MIRIN        B00000100
#define MASK_SAKE         B00001000
#define MASK_VINEGAR      B00010000
#define MASK_ROTARY_A     B00100000
#define MASK_ROTARY_B     B01000000
#define MASK_FRONTDISPENSE    B10000000
#define MASK_ALLSELECTLED    B00011110

#define GARDTIME           120000                           //@ 吐出時間ガードタイム(120s)
#define BTLMAX    300

#define LOGDESTINATION "http://your.log.dest"
#define LOGDESTINATION_PORT 80

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.nict.jp", 32400, 86400000); // UTC+9H, every 24H

MCP mcp(0, SPI_CS_PORT_NUM);
MCP mcpGPB(8, SPI_CS_PORT_NUM);

ESP8266WebServer server(80);

// ---------------------------------------------------------
// EEPROM
// β2：パラメータ変更に付き順番も変更
// ---------------------------------------------------------
#define EEPROMSIZE           4096
#define CONFIG_ADDR             0
#define REMAININGAMOUNT_ADDR   64
#define MOTORADJUST_ADDR       80
#define IPCONFIG_ADDR      144

struct CONFIG {                                             // 32x2 = 64byte
  char ssid[32];                                            // SSID    [YOUR_WIFI_SSID]
  char pass[32];                                            // PASSWORD[YOUR_WIFI_PASS]
};

struct REMAININGAMOUNT {                                    // 16byte
  int  btl[NUM_OF_BOTTLES];                                 // 各ボトル残量
};

struct MOTORADJUST {                                        //(4byte x 4) x 4bottle = 64byte
  int  thrRemainingAmount[NUM_OF_BOTTLES];                  //@ 閾値
  int  motorPower1[NUM_OF_BOTTLES];                         //@ 残量が閾値より大きい時の吐出パワー
  int  motorPower2[NUM_OF_BOTTLES];                         //@ 残量が閾値より小さい時の吐出パワー
  int  specificGravity[NUM_OF_BOTTLES];                     //@ 比重(100倍)
};

struct IPCONFIG {
  int staticIp[4];
  int gateway[4];
  int subnet[4];
};

CONFIG          g_syscfg;
REMAININGAMOUNT g_remainingamount;
MOTORADJUST     g_motoradjust;
IPCONFIG        g_ipconfig;

// ---------------------------------------------------------
// モーターと電磁弁による吐出関係
// ---------------------------------------------------------
const int g_nAddress[] = { 0x60, 0x61, 0x62, 0x63 };        // モータードライバのI2Cアドレス
bool g_startMotor = false;                                  // モーター駆動ON/OFF状態
int  g_motorCtrlIndex = 0;                                  // モーターをシーケンシャルに制御するためのインデックス番号
bool g_motorStatus[NUM_OF_BOTTLES];                         // モーター回転状態
int  g_motorRotateTime[NUM_OF_BOTTLES];                     // モーター回転時間
int  g_motorRotateCount[NUM_OF_BOTTLES];                    // タイマー割込み内でのモータ回転残り時間
int  g_amount_set = 25;                                     // 設定重量カウンタ判定値
int  g_amount_cnt;                                          // 設定重量カウンタ
int  g_amount[NUM_OF_BOTTLES];                              // モータごとの吐出量(ml)
int  g_weight[NUM_OF_BOTTLES];                              // ボトルごとの吐出重量(ml)
int  g_base_weight;                                         // 吐出重量のベースとする用 
bool g_ad_init = false;                                     // 重量AD初期取込フラグ    
int  g_prepare_wait;                                        // prepare時に電磁弁が開ききらない為のWait用
int  g_open_cnt;                                            // ソレノイドオープン時間カウンタ

#define MOVING_AVARAGE_MAX  100                             // 移動平均バッファ最大数
int     ad_num = 12;                                         // 移動平均設定値
int     g_ad_buf[MOVING_AVARAGE_MAX];                       // 重量AD値の平均化用           //@ 新設
int     g_motorPower;                                       // 現在のモーターパワー値覚え用 //@ 新設
int     g_TrimCount = 4;                                    // 最大,最小値を省くデータ個数
int     g_offset = 13;                                      // 吐出時の重量補正値

bool g_Log_NeedToBeSent = false;
bool g_Log_StandAlone = false;
bool g_Log_RoofDispensePushed = false;
bool g_Log_ContinuousDispense = false;

int g_nPrevKeyState_RoofDispense = 0;
int g_nPrevKeyState_FrontDispense = 0;
int g_nPrevKeyState_SoySource = 0;
int g_nPrevKeyState_Mirin = 0;
int g_nPrevKeyState_Sake = 0;
int g_nPrevKeyState_Vinegar = 0;
const int g_maskArray[NUM_OF_BOTTLES] = {MASK_SOYSOURCE, MASK_MIRIN, MASK_SAKE, MASK_VINEGAR};

#define STATE_IDLE 0
#define STATE_NETWORK_READY 1
#define STATE_NETWORK_DISPENSE 2
#define STATE_STANDALONE_READY 3
#define STATE_STANDALONE_DISPENSE 4
int g_State = STATE_IDLE;

char g_RcvBuffer[256];
int g_RcvBufferWrPtr;

bool g_LedState;
int g_selectLedState = 0;

Ticker ticker;    // 周期タイマー
unsigned long g_unCounter;
bool g_debug = false;

// -----------------------------------------
// シリアルコマンド用
// -----------------------------------------
bool g_isCmd_b = false;
int g_Cmd_b_motorId = 0;
int g_Cmd_b_motorPower = 0;
int g_Cmd_b_motorRotateTime = 3000;

// --------------------------------------------------------
// ロータリーエンコーダ用
// --------------------------------------------------------
#define COEFF_ENCVALUE 2
int  g_prevEnc;
int  g_encValue    = 7;                                     //@ 10msが走る前にこの値の正で点灯する。連続にセット。
int  g_encRawValue = 7 * COEFF_ENCVALUE;

// 種別選択状態
bool g_isSoySourceSelected = false;     // 醤油   ← 非選択
bool g_isMirinSelected     = false;     // みりん ← 非選択
bool g_isSakeSelected      = false;     // 酒     ← 非選択
bool g_isVinegarSelected   = false;     // 酢     ← 非選択

uint8_t g_StandAloneDispenseAmount = 0;

// ----------------------------------------------------------------------------------------------------------
// 関数
// ----------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------
// 状態設定処理
// ---------------------------------------------------------
void setState(int state) {
  g_State = state;
  Serial.printf("state: %d\n", state);
}

// ---------------------------------------------------------
// 状態取得処理
// ---------------------------------------------------------
int getState() {
  return g_State;
}

// ---------------------------------------------------------
// 種別選択全クリア処理
// ---------------------------------------------------------
void allSelectedFlgCancel() {
  g_isSoySourceSelected = false;
  g_isMirinSelected     = false;
  g_isSakeSelected      = false;
  g_isVinegarSelected   = false;
}
// -------------------------------------------------------------------
// インジケータ系関数
// -------------------------------------------------------------------
// ---------------------------------------------------------
// フロントインジケータ点灯処理
// ---------------------------------------------------------
void LedOn(void)
{
  uint16_t key = mcp.digitalRead();
  mcp.digitalWrite(0x7FFF & key);
  g_LedState = true;
}

// ---------------------------------------------------------
// フロントインジケータ消灯処理
// ---------------------------------------------------------
void LedOff(void)
{
  uint16_t key = mcp.digitalRead();
  mcp.digitalWrite(0x8000 | key);
  g_LedState = false;
}

// ---------------------------------------------------------
// フロントインジケータ点滅処理
// ---------------------------------------------------------
void ToggleLED( void )
{
  if ( g_LedState == true ) {
    LedOff();
  } else {
    LedOn();
  }
}

// ---------------------------------------------------------
// 種別インジケータ点滅処理
// ---------------------------------------------------------
void toggleSelectLed(int mask) {
  if ( (g_selectLedState & mask) == 0 ) {
    selectLedOff(mask);
    g_selectLedState |= mask;
  } else {
    selectLedOn(mask);
    g_selectLedState &= ~mask;
  }
}

// ---------------------------------------------------------
// 種別インジケータ全消灯処理
// ---------------------------------------------------------
void selectLedAllOff() {
  mcp.pinMode(0xF000); //input
}

// ---------------------------------------------------------
// 種別インジケータ消灯処理
// ---------------------------------------------------------
void selectLedOff(int mask) {
  mcp.pinMode(0xFF00); //Output
  uint16_t key = mcp.digitalRead();
  mcp.digitalWrite((mask << 7) | key);
}

// ---------------------------------------------------------
// 種別インジケータ点灯処理
// ---------------------------------------------------------
void selectLedOn(int mask) {
  mcp.pinMode(0xFF00); //Output
  uint16_t key = mcp.digitalRead();
  mcp.digitalWrite(~(mask << 7) & key);
}

// ---------------------------------------------------------
// 種別インジケータ全点灯処理
// ---------------------------------------------------------
void selectLedAllOn() {
  mcp.pinMode(0xFF00); //Output
  mcp.digitalWrite(~(MASK_ALLSELECTLED << 7) & (mcp.digitalRead() | 0x0F00));
}

// ---------------------------------------------------------
// 操作用LEDを全てOFF
// ---------------------------------------------------------
void controlPanelLedOff() {
  uint16_t keyGPA = mcp.digitalRead();
  selectLedAllOff();
  setAmountLed(0);
}

// ---------------------------------------------------------
// 電磁弁開放
// ---------------------------------------------------------
void solenoidOpen(){
  digitalWrite(SOLENOID, HIGH);
}

// ---------------------------------------------------------
// 電磁弁閉鎖
// ---------------------------------------------------------
void solenoidClose(){
  digitalWrite(SOLENOID, LOW);
}

// -------------------------------------------------------------------
// 処理系
// -------------------------------------------------------------------
// ---------------------------------------------------------
// グローバル変数の初期化
// ---------------------------------------------------------
void initVariable()
{
  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    g_motorStatus[i] = false;
    g_motorRotateTime[i] = 0;
    g_motorRotateCount[i] = 0;
    g_amount[i] = 0;
    g_weight[i] = 0;                                        //@ 設定重量 β2
  }

  g_base_weight = 0;                                        //@ 基準重量 β2
  g_startMotor = false;
  g_motorCtrlIndex = 0;
  memset(g_RcvBuffer, 0, sizeof(g_RcvBuffer));
  g_RcvBufferWrPtr = 0;

  g_LedState = false;
  g_unCounter = 0;

  uint16_t keyGPA = mcp.digitalRead();
  g_nPrevKeyState_RoofDispense = ((uint8_t)keyGPA & MASK_ROOFDISPENSE) == 0 ? LOW : HIGH; //最初のボタン押下状態を保持しておく
  g_nPrevKeyState_FrontDispense = ((uint8_t)keyGPA & MASK_FRONTDISPENSE) == 0 ? LOW : HIGH; //最初のボタン押下状態を保持しておく
  g_nPrevKeyState_SoySource = 0;
  g_nPrevKeyState_Mirin = 0;
  g_nPrevKeyState_Sake = 0;
  g_nPrevKeyState_Vinegar = 0;

}

// ---------------------------------------------------------
// Motor Controller
// I2C経由で指定したモータードライバのON/OFFを行う
// ---------------------------------------------------------
void motorControl(int motorNo, bool bOnOff )
{
  if ( motorNo >= 0 && motorNo < NUM_OF_BOTTLES ) {
    Wire.beginTransmission(g_nAddress[motorNo]);
    Wire.write(0x00);
    if ( bOnOff == true ) {
      g_motorPower = 0;

      if (g_isCmd_b) {
        g_motorPower = g_Cmd_b_motorPower;
      } else {
        if (g_remainingamount.btl[motorNo] < g_motoradjust.thrRemainingAmount[motorNo]) {
          g_motorPower = g_motoradjust.motorPower1[motorNo];
        } else {
          g_motorPower = g_motoradjust.motorPower2[motorNo];
        }
      }

      Serial.printf("Motor power:%d\n", g_motorPower);
      Wire.write((g_motorPower << 2) | 0x01); //回転PWM
      
    } else {
      Wire.write(0x00);
      //@ Serial.printf("g_amount[%d]:%d\n", motorNo, g_amount[motorNo]);
      Serial.printf("g_weight[%d]:%d\n", motorNo, g_weight[motorNo]);
      g_remainingamount.btl[motorNo] -= g_weight[motorNo];  //@ β2 モーター停止時に残量から吐出重量を減算する
    }
    int num = Wire.endTransmission();
  }
}
void motorChange(int motorNo)
{
  if ( motorNo >= 0 && motorNo < NUM_OF_BOTTLES ) {
    Wire.beginTransmission(g_nAddress[motorNo]);
    Wire.write(0x00);
    g_motorPower = 0;

    g_motorPower = g_motoradjust.motorPower2[motorNo];

    Serial.printf("Motor power:%d\n", g_motorPower);
    Wire.write((g_motorPower << 2) | 0x01);

    int num = Wire.endTransmission();
  }
}

// ---------------------------------------------------------
// URLエンコード処理
// ---------------------------------------------------------
String URLEncode(const char* msg)
{
  const char *hex = "0123456789abcdef";
  String encodedMsg = "";

  while (*msg != '\0') {
    if ( ('a' <= *msg && *msg <= 'z')
         || ('A' <= *msg && *msg <= 'Z')
         || ('0' <= *msg && *msg <= '9') ) {
      encodedMsg += *msg;
    } else {
      encodedMsg += '%';
      encodedMsg += hex[*msg >> 4];
      encodedMsg += hex[*msg & 15];
    }
    msg++;
  }
  return encodedMsg;
}

// ---------------------------------------------------------
// ログ送信処理
// ---------------------------------------------------------
void sendDispenseLog() {
  char  buffer[256];
  const String strTag[NUM_OF_BOTTLES] = {"b1", "b2", "b3", "b4"};
  const String strTagRemaining[NUM_OF_BOTTLES] = {"rb1", "rb2", "rb3", "rb4"};

  String str = "/dev/oicy_taste/dispensed?";

  //deviceid（MACアドレス）
  str += "deviceid=";
  WiFi.macAddress().toCharArray(buffer, 256);
  str += URLEncode(buffer);

  //ボトルごとの吐出量
  for (int i = 0; i < NUM_OF_BOTTLES; i++) { //アプリから送信
    str += "&" + strTag[i] + "=" + g_weight[i] * 1000; //uL単位に変換                               //@ β2 吐出重量を吐出量としてログ送信
  }

  //残量
  for (int i = 0; i < NUM_OF_BOTTLES; i++) {
    str += "&" + strTagRemaining[i] + "=" + g_remainingamount.btl[i] * 1000; //uL単位に変換
  }

  //天面ボタンか全面ボタンか
  str += "&btn=";
  if (g_Log_RoofDispensePushed) { //roof
    str += "roof";
  } else {
    str += "front";
  }

  //スタンドアローンか
  if (g_Log_StandAlone == true) {
    str += "&sa=1";
  }

  //連続吐出か
  if (g_Log_ContinuousDispense == true) {
    str += "&cd=1";
  }

  //時刻
  str += "&time=";
  getTimeStamp().toCharArray(buffer, 256);
  str += URLEncode(buffer);

  Serial.println(str);

  HTTPClient http;
  http.setTimeout(1000);  //暫定値

  http.begin(LOGDESTINATION + str); //HTTP

  ESP.wdtDisable();
  int httpCode = http.GET();
  ESP.wdtEnable(WDTO_0MS);
  
  if(httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);

    // file found at server
    if(httpCode == HTTP_CODE_OK) {
     String payload = http.getString();
     Serial.println(payload);
     Serial.println("log sent");
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

// ---------------------------------------------------------
// モーター駆動開始処理
// β2 : startMotor前に行っていた処理を集約
// ---------------------------------------------------------
void startMotor( void )
{
  int sm_state;
  
  sm_state = getState();                                    // 現在の状態を取得

  g_startMotor = false;                                     // モーター駆動ON/OFF状態 ← OFF

  delay(100);                                               //@ これは何の為の処理？

  if (sm_state == STATE_NETWORK_READY) {
    g_motorCtrlIndex = 0;
    if (g_amount[0] == 0) {                                 //@ 吐出しないボトルは動かさない(はじめのボトル決め)
      g_motorCtrlIndex = 1;
      if (g_amount[1] == 0) {
        g_motorCtrlIndex = 2;
        if (g_amount[2] == 0) {
          g_motorCtrlIndex = 3;
        }
      }
    }
    setState(STATE_NETWORK_DISPENSE);
    g_Log_StandAlone = false;
    Serial.println("Network Dispense Start");

  } else if (sm_state == STATE_STANDALONE_READY && readRotaryEncoder() == 7) {
    readHWSettings();                                       // 液種設定
    setState(STATE_STANDALONE_DISPENSE);
    Serial.println("Continuous Dispense Start");

  } else if (sm_state == STATE_STANDALONE_READY) {
    readHWSettings();                                       // 液種設定
    setState(STATE_STANDALONE_DISPENSE);
    g_Log_StandAlone = true;
    Serial.println("Stand Alone Dispense Start");
  }

  // 全ボトルのモーター回転時間セット
  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    g_motorRotateCount[i] = g_motorRotateTime[i];
    Serial.printf("Motor %d , %d msec\n" , i,  g_motorRotateCount[i]);
  }

  g_ad_init = false;                                        //@ AD格納初期化
  g_base_weight = getWeight();                              //@ 該当吐出重量に現在値(生値)を格納
  //@ 初回もn平均にするよう変更(次回きれいにしてください)
  for (int i=0; i<ad_num; i++) {
    getWeight();
    delay(50);
  }
  g_base_weight = getWeight();
  //@ ここまで
  solenoidClose();                                          //@ 電磁弁閉鎖
  motorControl(g_motorCtrlIndex, true);                     // 指定モーター駆動開始
  g_startMotor = true;                                      // モーター駆動ON/OFF状態 ← ON
}


// ---------------------------------------------------------
// モーター駆動停止処理
// ---------------------------------------------------------
void stopMotor( void )
{
  solenoidOpen();                                           //@ 電磁弁開放

  g_Log_NeedToBeSent = true;                      // ログ送信要求 ← ON
  g_startMotor = false;                           // モーター駆動ON/OFF状態 ← OFF

  // 残りのボトルの設定クリア
  int i = 0;
  for ( i; i < NUM_OF_BOTTLES; i++ ) {
    g_motorRotateCount[i] = 0;                    // モーター回転残り時間クリア
    motorControl(i, false);                       // モーター駆動停止
  }

  // 吐出停止時の状態遷移
  int state = getState();
  if (state == STATE_NETWORK_DISPENSE) {
    setState(STATE_IDLE);
  } else if (state == STATE_STANDALONE_DISPENSE) {
    setState(STATE_STANDALONE_READY);
  }
}

// ---------------------------------------------------------
// 全モーター駆動停止処理
// Just stop all the motors without updating any variarables
// ---------------------------------------------------------
void stopCompulsoryAllMotors(){
  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    Wire.beginTransmission(g_nAddress[i]);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();
  }
}

// ---------------------------------------------------------
// EEPROM 残量更新処理
// ---------------------------------------------------------
void writeRemainingAmountOnEEPROM() {
  EEPROM.put<REMAININGAMOUNT>(REMAININGAMOUNT_ADDR, g_remainingamount);
  EEPROM.commit();

  Serial.println("EEPROM: remaining");
}

// ---------------------------------------------------------
// 重量AD取得処理
// AD値の生値を返却する
// 範囲：0x0000～0x0FFF
// ---------------------------------------------------------
int getAD(void)
{
  int weight = 0;

//  Wire.requestFrom(0x4D,2);                                 // 1001 101 から 2byte 取得
  int ret = Wire.requestFrom(0x4D,2); // 1001 101 から 2byte 取得
//  Serial.printf("AD=(%d)\n", ret);  
  weight  = Wire.read();                                    // 1byte目取得(上位)
  weight  = weight << 8;
  weight |= Wire.read();                                    // 2byte目取得(下位)

  return weight;
}

// ---------------------------------------------------------
// 重量AD取得処理(平均値)
// 4平均のMAX,MINを除いた値を平均化
// ---------------------------------------------------------
int getWeight(void)
{
  int weight;
  int i;

  weight = getAD();

  // -------------------------------------------------------
  // 展開処理
  // -------------------------------------------------------
  if (g_ad_init == false) {
    // 初期
    for (i = 0; i < ad_num; i++) {
      g_ad_buf[i] = weight;
    }
    g_ad_init = true;
  } else {
    // 通常
    for (i = 0; i < ad_num; i++) {
      g_ad_buf[i] = g_ad_buf[i+1];
    }
    g_ad_buf[ad_num-1] = weight;
  }

  return trimmean(g_ad_buf, ad_num, g_TrimCount);
}

// ---------------------------------------------------------
// 最大値と最低値を省いた平均値を取得（再帰型）
// 
// 課題 ... 再帰型とした為、時間が掛かる。
//          dataを一時配列に入れて qsortするほうが良いと思われる。
// 
// data      > データ格納配列 ... 計算後値保証無し。
// length    > データ長
// trim_count> 最大値、最低値を取り除く個数
// ---------------------------------------------------------
int trimmean(int data[], int length, int trim_count)
{
    // <合計値、最大値、最低値を抽出>
    int  max, min, sum;
    int* pdata = data;

    pdata++;
    sum = max = min = data[0];
    for (int i=1; i<length; i++, pdata++) {
        sum += *pdata;
        if (max < *pdata) max = *pdata;
        if (min > *pdata) min = *pdata;
    }
    // </合計値、最大値、最低値を抽出>

    if (trim_count == 0) {
        // (最大値、最低値を取り除かない)
        return sum / length;
    } else if (trim_count == 1) {
        // (最大値、最低値は1つずつ取り除く)
        return (sum - max - min) / (length  - 2);
    } else {
        // (最大値、最低値を複数個取り除く)

        // <最大値、最小値を取り除いた配列を作り再帰呼び出し>
        int tmp[MOVING_AVARAGE_MAX];
        memset(tmp, 0, sizeof(tmp));

        bool  b_max = false;  // 最大値取り除いた？
        bool  b_min = false;  // 最小値取り除いた？
        int*  ptmp  = tmp;
        
        pdata = data;
        for (int i=0; i<length; i++, pdata++) {
            if (!b_max && (max == *pdata)) {
                // (最大値発見)
                b_max = true;
                continue;     // 除外
            }
            if (!b_min && (min == *pdata)) {
                // (最小値発見)
                b_min = true;
                continue;     // 除外
            }
            *ptmp = *pdata;   // 配列に組み入れる
            ptmp++;
        }
        return (trimmean(tmp, length-2, trim_count-1));
        // </最大値、最小値を取り除いた配列を作り再帰呼び出し>
    }
}

// ---------------------------------------------------------
// 重量計算処理
// AD値(生値)の差分をmlに変換して返す
// ---------------------------------------------------------
int calcWeight(int weight)
{
  weight  = weight - g_base_weight + g_offset;              // AD値生値同士の減算(MAX2046)
  weight *= 2319;                                           // g換算(0.2319g/Step)
  weight /= g_motoradjust.specificGravity[g_motorCtrlIndex];// 比重換算(100倍)
  weight /= 100;                                            // ml変換(小数点切捨て)(上記の余分の100で割る)

  return weight;
}

// ------------------------------------------------------------------
// Timer Handler
// ArduinoのMSTimer2は使えないので要注意。
// ESP8266のタイマーはシステムクロックのカウント値で設定する。
// 80MHzで駆動しているので、1秒は80000000Lとなる。
// この値から必要なカウント値を計算する
// ------------------------------------------------------------------
void timer0_ISR (void) {

  static int weight = 0;
  g_unCounter++;

//@debug
if(g_debug){
  if(g_unCounter % 5 == 0){
    //printf("AD : %d\n",getAD());
    printf("ave : %d\n",getWeight());
  }
}
//@debug

  // -------------------------------------------------------
  // モーター動作監視処理
  // -------------------------------------------------------
  if ( g_startMotor == true ) {
    g_open_cnt = 0;
    // -----------------------------------------------------
    // 重量監視
    // -----------------------------------------------------
    if(g_unCounter % 5 == 0){
      weight = getWeight();                                 // 吐出重量取得
      g_weight[g_motorCtrlIndex] = calcWeight(weight);      // 吐出重量差分取得(ml)
      if (g_weight[g_motorCtrlIndex]+7 >= g_amount[g_motorCtrlIndex]
        && g_motorPower != g_motoradjust.motorPower2[g_motorCtrlIndex]) { //@ 残りnmlになったらモーターパワーを変更
        motorChange(g_motorCtrlIndex);
      }
    }

    // -----------------------------------------------------
    // モーター or 状態制御
    // -----------------------------------------------------
    if (g_isCmd_b) {
      // ---------------------------------------------------
      // bコマンド
      // ---------------------------------------------------
      if (g_motorRotateCount[g_motorCtrlIndex] > 0) {
        g_motorRotateCount[g_motorCtrlIndex] -= 10;
        if (g_motorRotateCount[g_motorCtrlIndex] % SELECTLED_TOGGLE_INTERVAL < 10) {
          toggleSelectLed(g_maskArray[g_motorCtrlIndex]);
        }
      } else {
        solenoidOpen();
        motorControl(g_motorCtrlIndex, false);              // 指定したモーターを停止する。
        g_startMotor = false;                               // モーター駆動ON/OFF状態 ← OFF
        setState(STATE_IDLE);
        controlPanelLedOff();
      }
    } else if ( getState() == STATE_STANDALONE_DISPENSE && readRotaryEncoder() == 7 ) {
      // ---------------------------------------------------
      // 指定モーター連続吐出中 ログ用吐出量加算処理 & LED制御
      // ---------------------------------------------------
      g_motorRotateCount[g_motorCtrlIndex] += 10;           //@ 点滅制御用に処理を残す。吐出量として使うことはない。
      if (g_motorRotateCount[g_motorCtrlIndex] % SELECTLED_TOGGLE_INTERVAL < 10) {
        toggleSelectLed(g_maskArray[g_motorCtrlIndex]);
      }

    } else if ( getState() != STATE_STANDALONE_READY
        && g_motorRotateCount[g_motorCtrlIndex] > 0                 //@ 時間制御にて使用していたもの。ガードタイマとして流用する。
        && (checkWait(
                g_weight[g_motorCtrlIndex], 
                g_amount[g_motorCtrlIndex], 
               &g_amount_cnt, 
                g_amount_set) == false)                             //@ 吐出重量 < 設定重量
        && g_amount[g_motorCtrlIndex] > 0                           //@ 設定重量 != なし
        && g_prepare_wait == 0 ) {                                  //@ wait中ではない
      // ---------------------------------------------------
      // 指定モーター吐出中 吐出時間減算処理 & LED制御
      // ---------------------------------------------------
      g_motorRotateCount[g_motorCtrlIndex] -= 10;
      if (g_motorRotateCount[g_motorCtrlIndex] % SELECTLED_TOGGLE_INTERVAL < 10) {
        toggleSelectLed(g_maskArray[g_motorCtrlIndex]);
      }
      g_prepare_wait = 0;

    } else if ( getState() == STATE_STANDALONE_DISPENSE ) {
      // ---------------------------------------------------
      // 手動吐出終了(連続以外：連続吐出はloop内で終了する)
      // ---------------------------------------------------
      Serial.printf("Stop motor %d\n", g_motorCtrlIndex);
      if (g_isCmd_b) {
        motorControl(g_motorCtrlIndex, false);              // 指定したモーターを停止する。
        g_startMotor = false;                               // モーター駆動ON/OFF状態 ← OFF
        setState(STATE_STANDALONE_READY);
        g_motorCtrlIndex = 0;
      } else {
        stopMotor();
        writeRemainingAmountOnEEPROM();
      }
      selectLedOn(g_maskArray[getSelectedId()]);            // 該当の液種のみ再度点灯
      printf("g_amount:%d , g_weight:%d , g_base_weight:0x%x , weight:0x%x , step:%d\n",g_amount[g_motorCtrlIndex], g_weight[g_motorCtrlIndex],g_base_weight,weight,weight-g_base_weight);

    } else {
      // ---------------------------------------------------
      // prepare時、指定モーター吐出終了
      // ---------------------------------------------------
      if (g_prepare_wait == 0) {
        Serial.printf("Stop motor %d\n", g_motorCtrlIndex);
        solenoidOpen();                                     //@ 電磁弁開放
        motorControl(g_motorCtrlIndex, false);              // 指定したモーターを停止する。
        selectLedOff(g_maskArray[g_motorCtrlIndex]);        // 該当のLEDを消灯
        g_motorCtrlIndex++;                                 // 指定モーターを次のモーターに移行

        for (int i=g_motorCtrlIndex; i<4; i++) {            //@ 吐出しないボトルは動かさない
          if (g_motorCtrlIndex < NUM_OF_BOTTLES             //@ ボトル範囲内
            && g_amount[g_motorCtrlIndex] == 0) {           //@ && 吐出なし
            g_motorCtrlIndex++;                             //@ 指定モーターを次のモーターに移行
          } else {
            break;
          }
        }
      }
      if ( g_motorCtrlIndex >= NUM_OF_BOTTLES ) {
        // -------------------------------------------------
        // prepare時、全吐出終了
        // -------------------------------------------------
        g_Log_NeedToBeSent = true;
        writeRemainingAmountOnEEPROM();
        Serial.printf("State: %d\n", getState());
        setState(STATE_IDLE);                               //状態を基に戻す
        g_startMotor = false;
        g_motorCtrlIndex = 0;
        g_prepare_wait = 0;                                 //@ waitｶｳﾝﾀﾘｾｯﾄ

      } else {
        // -------------------------------------------------
        // prepare時、全吐出終了まだ
        // 次のモーター駆動開始
        // -------------------------------------------------
        g_prepare_wait++;
        g_base_weight = getWeight();                        //@ 該当吐出重量に現在値(生値)を格納
        if (g_prepare_wait > 200) {                         //@ 次の吐出までにWaitを入れないと前のが出続ける
          solenoidClose();                                  //@ 電磁弁閉鎖
          motorControl(g_motorCtrlIndex, true);
          g_prepare_wait = 0;                               //@ waitｶｳﾝﾀﾘｾｯﾄ
        }
      }
    }
  } else {
    g_open_cnt++;                                           //@ モーター不動作時のソレノイド開閉時間監視
    if (g_open_cnt > 1000) {
      solenoidClose();
    }
  }

  // -------------------------------------------------------
  // ロータリーエンコーダー更新処理
  // -------------------------------------------------------
  if (getState() == STATE_STANDALONE_READY) {
    updateRotaryEncoder();
  }

  // -------------------------------------------------------
  // 全モーター駆動停止判定(たぶん安全措置)
  // -------------------------------------------------------
  int state = getState();
  if(state != STATE_NETWORK_DISPENSE && state != STATE_STANDALONE_DISPENSE && g_isCmd_b == false){
    stopCompulsoryAllMotors();
  }

  // -------------------------------------------------------
  // Wifi途絶中LED処理
  // -------------------------------------------------------
  if(WiFi.status() != WL_CONNECTED){
    if(g_unCounter % 10 == 0){
      ToggleLED();
    }
  } else {
    LedOn();
  }
}

// ---------------------------------------------------------
// 重量判定（カウンタ付き）
// 重量の設定重量に対する超過が、設定回数に到達したかを含めて
// 判定。
//
// 2018/04/11現在は、回数超過の連続性は未対応。
// ひとまず単純なカウントのみ。
//
// weight    > 現在の重量                              (i)
// set_weight> 設定重量                                (i)
// count     > 設定重量を超えた回数                    (i/o)
// set_count > 設定重量を超える回数の設定値            (i)
//
// Return : true  = 重量が設定重量を超えた（回数も設定に到達）。
//          false = 重量が設定重量未満（もしくは、回数未達）
// ---------------------------------------------------------
bool checkWait(int weight, int set_weight, int* count, int set_count)
{
    if (weight <= set_weight) {
        *count = 0;      // <-- カウンタクリア必要か検討。
        return false;
    }

    (*count)++;
    return (*count >= set_count) ? true : false;
}

// ---------------------------------------------------------
// ルート接続表示
// ---------------------------------------------------------
void handleRoot() {
  server.send(200, "text/html", "<h1>OiCy Taste beta2 Firmware</h1>");
}

// ---------------------------------------------------------
// ステータス表示
// ---------------------------------------------------------
void handleStatus() {

  String message = "<html><head></head>";
  message += "<body>";

  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    message += "Move time : Motor " + String(i) + " : " + String(g_motorRotateTime[i]) + "<br>";
  }

  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    message += "Count : Motor " + String(i) + " : " + String(g_motorRotateCount[i]) + "<br>";
  }

  for ( int i = 0; i < NUM_OF_BOTTLES; i++) {
    if (g_motorStatus[i] == true) {
      message += "Status : Motor " + String(i) + " : " + "ON" + "<br>";

    } else {
      message += "Status : Motor " + String(i) + " : " + "OFF" + "<br>";
    }
  }
  message += "</body></html>";


  server.send(200, "text/html", message);
}

// ---------------------------------------------------------
// prepare
// 取り替え時か通常時かの判断を行い、取り替え時のときは、HistoryをClearする
// ---------------------------------------------------------
void handlePrepare() {
  int amount;
  Serial.print("prepare\n");
  const String strTag[NUM_OF_BOTTLES] = {"b1", "b2", "b3", "b4"};
  const String strTagReset[NUM_OF_BOTTLES] = {"rb1", "rb2", "rb3", "rb4"};
  const String strRst = {"r"};

  //H/Wボタンの設定をリセットする
  //操作用LEDを全てOff
  controlPanelLedOff();

  if (server.hasArg(strRst)) {
    Serial.print("Reset\n");
    //rがあった場合、指定されたボトルのみリセットする
    for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
      g_amount[i] = 0;
      if (server.hasArg(strTagReset[i])) {
        amount = server.arg(strTagReset[i]).toInt();
        Serial.printf("Reset b%d to %d \n", i + 1, amount);
        g_remainingamount.btl[i] = amount / 1000; //uL -> mL
      }
    }
    writeRemainingAmountOnEEPROM();
  } else {
    Serial.print("Not Reset\n");
  }
  handleSettings();

  setState(STATE_NETWORK_READY);
}


// ---------------------------------------------------------
// prepare 吐出量の設定
// ※もしTTL吐出量がBTLMAXを超えてたら、吐出量は0に設定
// 返り値：いままでの吐出量の合計
// ---------------------------------------------------------
void handleSettings() {

  String motor[NUM_OF_BOTTLES];
  const String strTag[NUM_OF_BOTTLES] = {"b1", "b2", "b3", "b4"};
  const String strTagRemaining[NUM_OF_BOTTLES] = {"rb1", "rb2", "rb3", "rb4"};
  char  buffer[64];

  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    // ここでクライアントから来たデータを確認する。
    motor[i] = server.arg(strTag[i]);
    motor[i].toCharArray( buffer, 64 );
    g_amount[i] = atoi( buffer);
    int mod = ( g_amount[i] / 100 ) % 10;  // 0.5mmの値を計算する
    g_amount[i] /= 1000;    // mlに換算
    if ( mod >= 5 ) {       // 0.1mlのくらいを四捨五入する。
      g_amount[i]++;
    }
    //LED表示
    if (g_amount[i] > 0) {
      selectLedOn(g_maskArray[i]);
    }
    //weight初期化
    g_weight[i] = 0;

    Serial.printf("g_amount[%d]=%d\n", i, g_amount[i]);

    //TTL吐出量がBTLMAXを超えてたら吐出しないようにする
    if ( g_remainingamount.btl[i] > BTLMAX ) {
      g_motorRotateTime[i] = 0;
    } else {
      g_motorRotateTime[i] = GARDTIME;                      //@ s固定
    }
    Serial.printf("g_motorRotateTime[%d]:%d\n", i, g_motorRotateTime[i]);
  }

  //handleSettingの返り値
  String message = "<html><head></head>";
  message += "<body>";

  message += "<div class=\"total_amount\">\n";
  for ( int i = 0; i < NUM_OF_BOTTLES; i++) {
    message += "<div id=" + strTag[i] + ">" + "\n";
    message += strTag[i] + ":" + "<span class=\"amount\">" + g_amount[i] * 1000 + "</span><span class=\"unit\">uL</span>" + "\n";
    message += "</div>\n";
  }
  for ( int i = 0; i < NUM_OF_BOTTLES; i++) {
    message += "<div id=" + strTagRemaining[i] + ">" + "\n";
    message += strTagRemaining[i] + ":" + "<span class=\"amount\">" + g_remainingamount.btl[i] * 1000 + "</span><span class=\"unit\">uL</span>" + "\n";
    message += "</div>\n";
  }

  message += "</div>\n";

  // Loadcell Value
  for ( int i = 0; i < NUM_OF_BOTTLES; i++) {
    message += ( strTag[i] + " : " + motor[i] + "</br>" );
  }
  message += "'<br />";
  message += "<form method='get' action='./prepare'>";

  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    // OFF
    message += strTag[i] + ":" + "<input type='text' name='" + strTag[i] + "' value='" + motor[i] + "' ";
    message += "/>Amount<BR>";

  }

  message += "<input type='submit' value='send'>";
  message += "</button>";
  message += "</form>";
  message += "</body></html>";

  server.send(200, "text/html", message);

}


// ---------------------------------------------------------
// WiFi設定画面表示(GET)
// ---------------------------------------------------------
void handleNetCfg_Get() {

  String html = "<html><head></head>";
  html += "<body>";
  html += "<h1>WiFi Settings</h1>";
  html += "<form method='post'>";
  html += "  <input type='text' name='ssid' placeholder='ssid'><br>";
  html += "  <input type='text' name='pass' placeholder='pass'><br>";
  html += "  <input type='submit'><br>";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);

}

// ---------------------------------------------------------
// WiFi設定画面表示(POST)
// ---------------------------------------------------------
void handleNetCfg_Post() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");

  ssid.toCharArray(g_syscfg.ssid, 32);
  pass.toCharArray(g_syscfg.pass, 32);

  EEPROM.put<CONFIG>(CONFIG_ADDR, g_syscfg);
  EEPROM.commit();

  String html = "";
  html += "<h1>WiFi Settings</h1>";
  html += ssid + "<br>";
  html += pass + "<br>";
  server.send(200, "text/html", html);
}

// ---------------------------------------------------------
// ソフト吐出ボタン
// ---------------------------------------------------------
void handlePushButton(){

    //H/Wの値を読み取るとHIGHなので、LOW → HIGHになったように見える
    g_nPrevKeyState_RoofDispense = LOW;

    String message = "<html><head></head>";
    message += "<body>";
    message += "<h1>Push button</h1>";
    message += "<form method='get' action='./pushbutton'>";
    message += "<input type='submit' value='send'>";
    message += "</button>";
    message += "</form>";
    message += "</body></html>";
    server.send(200, "text/html", message);
}

// ---------------------------------------------------------
// モーター回転開始
// β2 : STATE_NETWORK_READYのみ動作するように変更
// ---------------------------------------------------------
void handleStart() {
  const String strTag[NUM_OF_BOTTLES] = {"b1", "b2", "b3", "b4"};

  String message = "<html><head></head>";
  message += "<body>";

  if (getState() == STATE_NETWORK_READY) {
    startMotor();
    
    message += "<h1>Start</h1>";
    message += "dispensed?";
    for ( int i = 0; i < NUM_OF_BOTTLES; i++) {
      message += ( strTag[i] + "=" + g_amount[i] + "&");
    }
  } else {
    message += "<h1>Can't Start</h1>";
  }
  message += "time=" + getTimeStamp();
  message += "</body></html>";
  server.send(200, "text/html", message);

}

// ---------------------------------------------------------
// モーター回転停止
// ---------------------------------------------------------
void handleStop() {
  stopMotor();

  server.send(200, "text/html", "<h1>Stop</h1>");
}


// ---------------------------------------------------------
// シリアル受信パース処理
// ---------------------------------------------------------
void ParseCommand( char *pCommand )
{
  char *pTmpPtr = pCommand;

  if ( *pTmpPtr == 'r' ) {
    // -----------------------------------------------------
    // ソフトリセット
    // -----------------------------------------------------
    g_startMotor = false;
    for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
      motorControl(i, false);
    }
    ESP.reset();

  } else if ( *pTmpPtr == 'b' ) {
    // -----------------------------------------------------
    // モーター回転コマンド('b')
    // -----------------------------------------------------
    g_isCmd_b = true;
    const String strTag[NUM_OF_BOTTLES] = {"b1", "b2", "b3", "b4"};
    pTmpPtr++;
    if ( *pTmpPtr >= '1' && *pTmpPtr <= '4' ) {
      g_Cmd_b_motorId = *pTmpPtr - '1';
      pTmpPtr++;
    } else if ( !(*pTmpPtr == ',' || *pTmpPtr == '\0')) { //","と終端以外はエラー
      Serial.println("=> ERROR: Bottle ID must be between 1 and 4");
      return;
    }

    char buf[128];
    strcpy(buf, pTmpPtr);
    char *c;
    int idx = 0;
    int param[2]; //モーターパワー、駆動時間の２つ
    for (c = strtok(buf, ","); c; c = strtok(NULL, ",")) {
      param[idx] = atoi(c);
      //          Serial.printf("%d ", param[idx]);
      idx++;
    }
    //        Serial.println();

    if (idx == 1) {
      g_Cmd_b_motorRotateTime = param[0];
    } else if (idx == 2) {
      g_Cmd_b_motorPower = param[0];
      g_Cmd_b_motorRotateTime = param[1];
    }

    Serial.printf("Start Motor: %s, power:%d, %dmsec\n", strTag[g_Cmd_b_motorId].c_str(), g_Cmd_b_motorPower, g_Cmd_b_motorRotateTime);
    g_startMotor = false;
    delay(100);
    g_motorRotateCount[g_Cmd_b_motorId] = g_Cmd_b_motorRotateTime;
    g_motorCtrlIndex = g_Cmd_b_motorId;
    allSelectedFlgCancel();
    controlPanelLedOff();
    solenoidClose();
    motorControl(g_motorCtrlIndex, true);
    g_startMotor = true;

  } else if ( *pTmpPtr == 'w' ) {
    // -----------------------------------------------------
    // 補正値セットコマンド('w')  weight
    // -----------------------------------------------------
    pTmpPtr++;
    if ( *pTmpPtr >= '1' && *pTmpPtr <= '4' ) {
      int nMotor = *pTmpPtr - '1';
      pTmpPtr++;

      if ( *pTmpPtr == ',' ) {
        char buf[128];
        pTmpPtr++;
        strcpy(buf, pTmpPtr);

        char *c;
        int idx = 0;
        int param[4]; //パラメータ数 4
        for (c = strtok(buf, ","); c; c = strtok(NULL, ",")) {
          param[idx] = atoi(c);
          Serial.printf("%d ", param[idx]);
          idx++;
        }
        Serial.println();

        if (idx < 4) {
          Serial.println("ERROR: All parameters are needed.");
        } else {
          Serial.println("Write to EEPROM");
          g_motoradjust.thrRemainingAmount[nMotor] = param[0];
          g_motoradjust.motorPower1[nMotor] = param[1];
          g_motoradjust.motorPower2[nMotor] = param[2];
          g_motoradjust.specificGravity[nMotor] = param[3];
          EEPROM.put<MOTORADJUST>(MOTORADJUST_ADDR, g_motoradjust);
          EEPROM.commit();
        }

      } else {
        Serial.println("Command Error");
        return;
      }
    } else {
      Serial.println("Weight info:");
      Serial.println("Bottle:threshold, power1, power2, specificgravity");
      for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
        Serial.printf("b%d:%d,%d,%d,%d\n",
                      i + 1,
                      g_motoradjust.thrRemainingAmount[i],
                      g_motoradjust.motorPower1[i],
                      g_motoradjust.motorPower2[i],
                      g_motoradjust.specificGravity[i]
                     );
      }
      return;
    }
  } else if ( *pTmpPtr == 't' ) {
    // -----------------------------------------------------
    // TTL吐出量セットコマンド('t')
    // -----------------------------------------------------
    pTmpPtr++;
    
    if ( *pTmpPtr >= '1' && *pTmpPtr <= '4' ) { //ボトルの指定がある場合
      int nMotor = *pTmpPtr - '1';
      pTmpPtr++;
      
      //カンマの後の値を書き込む。その後にカンマがある場合のエラー処理はしない（指定NG！）
      if ( *pTmpPtr == ',' ) {
        pTmpPtr++;
        int nTtl = atoi(pTmpPtr);
        g_remainingamount.btl[nMotor] = nTtl;
        Serial.printf("Write EEPROM TTL amount\n");
        Serial.printf(" b%d: %d mL\n", nMotor+1, nTtl);
        writeRemainingAmountOnEEPROM();

      } else {
        Serial.println("Command Error");
        return;
      }

    } else if(*pTmpPtr == 'r'){ //"tr"の時は250,250,250,200でリセット

        Serial.printf("Write EEPROM TTL amount with 250,250,250,200\n");
        g_remainingamount.btl[0] = 250;
        g_remainingamount.btl[1] = 250;
        g_remainingamount.btl[2] = 250;
        g_remainingamount.btl[3] = 200;
        writeRemainingAmountOnEEPROM();
        return;
    } else if ( *pTmpPtr == ',') { //カンマ区切りで続く場合

      //カンマ区切りで分割
        char buf[128];
        strcpy(buf, pTmpPtr);
        char *c;
        int idx = 0;
        int param[NUM_OF_BOTTLES]; //モーター数
        for (c = strtok(buf, ","); c; c = strtok(NULL, ",")) {
          param[idx] = atoi(c);
          idx++;
        }
        
        if(idx < NUM_OF_BOTTLES){
          Serial.println("Command Error");
          return;
        }else{
          Serial.printf("Write EEPROM TTL amount\n");
          for(int i; i < NUM_OF_BOTTLES; i++){
            Serial.printf(" b%d: %d mL\n", i+1, param[i]);
            g_remainingamount.btl[i] = param[i];
          }
          writeRemainingAmountOnEEPROM();
        }
  
      } else if ( *pTmpPtr == '\0') { //t単体は残量表示
  
        EEPROM.get<REMAININGAMOUNT>(REMAININGAMOUNT_ADDR, g_remainingamount);
  
        Serial.print("Bottle TTL Amount\n");
  
        for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
          Serial.printf(" b%d:%d mL\n", i+1, g_remainingamount.btl[i]);
         }
        Serial.print("\n");
        return;
  
      } else { //エラー
        Serial.println("=> ERROR: Bottle ID must be between 1 and 4");
        return;
      }
      
  } else if ( *pTmpPtr == 's' ) {
    // -----------------------------------------------------
    // ネットワーク状態表示コマンド('s')
    // -----------------------------------------------------
    //状態表示コマンド("st")
    if ( *(pTmpPtr + 1) == 't') {
      Serial.print("State : ");
      Serial.println(getState());
      return;
    } else if (*(pTmpPtr + 1) == 'n') {
      //サブネットの変更("sn")
      pTmpPtr++;
      pTmpPtr++;
      if ( *pTmpPtr == ',' ) {
        pTmpPtr++;
        int newSubnet[4];
        String item;
        int len = String(pTmpPtr).length();

        int j = 0;
        for (int i = 0; i < len; i++) {
          String s = String(pTmpPtr[i]);
          if (s.compareTo(".") == 0) {
            if (item != "") {
              newSubnet[j] = item.toInt();
              j++;
              item = "";
            }
          }
          else {
              item.concat(s);
          }
        }
        if (item != "") {
          newSubnet[j] = item.toInt();
        }

        memset(g_ipconfig.subnet, 0, sizeof(g_ipconfig.subnet));
        memcpy(g_ipconfig.subnet, newSubnet, sizeof(g_ipconfig.subnet));
        EEPROM.put<IPCONFIG>(IPCONFIG_ADDR, g_ipconfig);
        EEPROM.commit();
        Serial.print("now subnet is ");
        Serial.println(String(g_ipconfig.subnet[0]) + "." + String(g_ipconfig.subnet[1]) + "." + String(g_ipconfig.subnet[2]) + "." + String(g_ipconfig.subnet[3]));
        Serial.print("Please reset if you want to reflect new subnet");
      }
    } else {
      EEPROM.get<CONFIG>(CONFIG_ADDR, g_syscfg);

      Serial.print("WiFi SSID : ");
      Serial.println(g_syscfg.ssid);

      Serial.print("WiFi PASS : ");
      Serial.println(g_syscfg.pass);

      Serial.print("Network State : ");
      Serial.println(WiFi.status());

      Serial.print("IP Address : ");
      Serial.println(WiFi.localIP());
      Serial.print("Gateway IP:");
      Serial.println(address(WiFi.gatewayIP()));
      Serial.print("SubnetMask:");
      Serial.println(address(WiFi.subnetMask()));
    }
  } else if ( *pTmpPtr == 'a' ) {   // Wifi AP SSID
    // -----------------------------------------------------
    // アクセスポイントSSIDセットコマンド('a')
    // -----------------------------------------------------
    pTmpPtr++;
    if ( *pTmpPtr == ',' ) {
      pTmpPtr++;
      memset( g_syscfg.ssid, 0, sizeof(g_syscfg.ssid));
      strncpy(g_syscfg.ssid, pTmpPtr , 32);

      Serial.print("Write EEPROM (AP SSID) : ");
      Serial.println(g_syscfg.ssid);

      EEPROM.put<CONFIG>(CONFIG_ADDR, g_syscfg);
      EEPROM.commit();


    } else {
      Serial.println("Command Error");
      return;
    }

  } else if ( *pTmpPtr == 'p' ) {   // Wifi AP Password
    // -----------------------------------------------------
    // アクセスポイントpasswordセットコマンド('p')
    // -----------------------------------------------------
    pTmpPtr++;
    if ( *pTmpPtr == ',' ) {
      pTmpPtr++;

      memset( g_syscfg.pass, 0, sizeof(g_syscfg.pass));
      strncpy(g_syscfg.pass, pTmpPtr , 32 );;

      Serial.print("Write EEPROM (AP PASS) : ");
      Serial.println(g_syscfg.pass);

      EEPROM.put<CONFIG>(CONFIG_ADDR, g_syscfg);
      EEPROM.commit();

    } else {
      Serial.println("Command Error");
      return;
    }
  } else if ( *pTmpPtr == 'g' ) {
    // -----------------------------------------------------
    // g単発でﾍﾞｰｽ重量測定(β2デバッグ用)
    // g1 で差分表示(モーターインデックスは通常0：醤油)
    // gw でgatewayの変更
    // -----------------------------------------------------
    if (*(pTmpPtr + 1) == '1') {
      getWeight();
      delay(50);
      getWeight();
      delay(50);
      getWeight();
      delay(50);
      int weight = getWeight();
      Serial.printf("base_step : 0x%x / weight : %dg\n",g_base_weight,calcWeight(weight));
    } else if (*(pTmpPtr + 1) == 'w') {
      pTmpPtr++;
      pTmpPtr++;
      if ( *pTmpPtr == ',' ) {
        pTmpPtr++;
        int newGateway[4];
        String item;
        int len = String(pTmpPtr).length();

        int j = 0;
        for (int i = 0; i < len; i++) {
          String s = String(pTmpPtr[i]);
          if (s.compareTo(".") == 0) {
            if (item != "") {
              newGateway[j] = item.toInt();
              j++;
              item = "";
            }
          }
          else {
              item.concat(s);
          }
        }
        if (item != "") {
          newGateway[j] = item.toInt();
        }

        memset(g_ipconfig.gateway, 0, sizeof(g_ipconfig.gateway));
        memcpy(g_ipconfig.gateway, newGateway, sizeof(g_ipconfig.gateway));
        EEPROM.put<IPCONFIG>(IPCONFIG_ADDR, g_ipconfig);
        EEPROM.commit();
        Serial.print("now gateway is ");
        Serial.println(String(g_ipconfig.gateway[0]) + "." + String(g_ipconfig.gateway[1]) + "." + String(g_ipconfig.gateway[2]) + "." + String(g_ipconfig.gateway[3]));
        Serial.print("Please reset if you want to reflect new gateway");
      }
    } else {
      g_ad_init = false;                                      //@ AD格納初期化
      g_base_weight = getWeight();
      Serial.printf("base_step : 0x%x\n",g_base_weight);
    }
  } else if ( *pTmpPtr == 'd' ) {
    // -----------------------------------------------------
    // AD生値出力用
    // -----------------------------------------------------
    if (*(pTmpPtr + 1) == '1') {
      g_debug = true;
    } else {
      g_debug = false;
    }
  } else if ( *pTmpPtr == 'i' ) {
    // -----------------------------------------------------
    // 静的IPアドレスの変更('ip')
    // -----------------------------------------------------
    pTmpPtr++;
    if ( *pTmpPtr == 'p' ) {
      pTmpPtr++;
      if ( *pTmpPtr == ',' ) {
        pTmpPtr++;
        int newIp[4];
        String item;
        int len = String(pTmpPtr).length();

        int j = 0;
        for (int i = 0; i < len; i++) {
          String s = String(pTmpPtr[i]);
          if (s.compareTo(".") == 0) {
            if (item != "") {
              newIp[j] = item.toInt();
              j++;
              item = "";
            }
          }
          else {
              item.concat(s);
          }
        }
        if (item != "") {
          newIp[j] = item.toInt();
        }

        memset(g_ipconfig.staticIp, 0, sizeof(g_ipconfig.staticIp));
        memcpy(g_ipconfig.staticIp, newIp, sizeof(g_ipconfig.staticIp));
        EEPROM.put<IPCONFIG>(IPCONFIG_ADDR, g_ipconfig);
        EEPROM.commit();
        Serial.print("now static ip address is ");
        Serial.println(String(g_ipconfig.staticIp[0]) + "." + String(g_ipconfig.staticIp[1]) + "." + String(g_ipconfig.staticIp[2]) + "." + String(g_ipconfig.staticIp[3]));
        Serial.print("Please reset if you want to reflect new ip address");
      }
    }
  }
}

String address(IPAddress ip){
  String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
  return ipStr;
}

// ---------------------------------------------------------
// タイムスタンプ作成処理（ログ送信時）
// ---------------------------------------------------------
String getTimeStamp() {
  timeClient.update();
  setTime(timeClient.getEpochTime());
  return (String)year() + "-" + (month() < 10 ? "0" : "") + month() + "-" + (day() < 10 ? "0" : "") + day() + " " + (hour() < 10 ? "0" : "") + hour() + ":" + (minute() < 10 ? "0" : "") + minute() + ":" + (second() < 10 ? "0" : "") + second() + " +0900";
}

// ---------------------------------------------------------
// Initialize GPIO Expander
// ---------------------------------------------------------
void initMCP23S17() {
  mcp.begin();//x.begin(1) will override automatic SPI initialization
  mcp.digitalWrite(0xFF00 | mcp.digitalRead());             //@ LED全消灯
  mcp.pinMode(0xFF00);                                      //@ I/O設定
}

// ---------------------------------------------------------
// Initial処理
// ---------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  Serial.println("");
  Serial.println("Start");
  Serial.printf("Ver: %s\n", VERSION);
  Serial.printf("size of CONFIG:%d\n", sizeof(CONFIG));
  Serial.printf("size of REMAININGAMOUNT:%d\n", sizeof(REMAININGAMOUNT));
  Serial.printf("size of MOTORADJUST:%d\n", sizeof(MOTORADJUST));

  // Initialize GPIO
  pinMode(BOOTMODE_BUTTON, INPUT);
  digitalWrite(SOLENOID, HIGH);                             //@ ボトル入れ替えとかの圧を抜くために一旦OPEN
  pinMode(SOLENOID, OUTPUT);

  // Initialize EEPROM
  EEPROM.begin(EEPROMSIZE);

  EEPROM.get<CONFIG>(CONFIG_ADDR, g_syscfg);
  EEPROM.get<REMAININGAMOUNT>(REMAININGAMOUNT_ADDR, g_remainingamount);
  EEPROM.get<MOTORADJUST>(MOTORADJUST_ADDR, g_motoradjust);
  EEPROM.get<IPCONFIG>(IPCONFIG_ADDR, g_ipconfig);

  //　MCP23S17初期化
  initMCP23S17();

  // グローバル変数の初期化
  initVariable();

  // 状態遷移初期化
  setState(STATE_IDLE);

  // -------------------------------------------------------
  //  クライアントモード。接続先をEPROMから読み出す。
  //   WiFi接続タイムアウト処理付き
  // -------------------------------------------------------

  // AP設定をEEPROMから読み出すVersion
  int count = 0;

  // EEPROMに設定が
  if ( g_syscfg.ssid[0] != 0xff && g_syscfg.pass[0] != 0xff ) {
    int count = 0;
    Serial.printf("Connecting to %s\n", g_syscfg.ssid);

    WiFi.begin(g_syscfg.ssid, g_syscfg.pass);

    while (true)
    {
      if ( WiFi.status() != WL_CONNECTED ) {
        ToggleLED();
        delay(500);
        Serial.print(".");
      } else {
        IPAddress ip(g_ipconfig.staticIp[0], g_ipconfig.staticIp[1], g_ipconfig.staticIp[2], g_ipconfig.staticIp[3]);
        IPAddress gateway(g_ipconfig.gateway[0], g_ipconfig.gateway[1], g_ipconfig.gateway[2], g_ipconfig.gateway[3]);
        IPAddress subnet(g_ipconfig.subnet[0], g_ipconfig.subnet[1], g_ipconfig.subnet[2], g_ipconfig.subnet[3]);

        WiFi.config(ip, gateway, subnet);

        LedOn();
        Serial.println(" connected");
        Serial.println(WiFi.localIP());
        Serial.println(WiFi.macAddress());
        Serial.println(getTimeStamp());

        break;
      }
      count++;

      if ( count > 30 ) {//15秒
        Serial.println("Wifi connection timeout.\n");
        break;
      }
    }
  }

  WiFi.setOutputPower(0); // WiFi出力制限
  
  Wire.begin(4, 5);               // Wire.begen(SDA,SCL) ← I/Oポート設定
  delay(40);
  motorControl(0, false);
  motorControl(1, false);
  motorControl(2, false);
  motorControl(3, false);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/prepare", handlePrepare );
  //server.on("/btlInitialize",**** );
  server.on("/pushbutton", handlePushButton );
  server.on("/start", handleStart );
  server.on("/stop", handleStop );
  server.on("/network", HTTP_GET, handleNetCfg_Get );
  server.on("/network", HTTP_POST, handleNetCfg_Post );

  server.begin();
  solenoidClose();                                          //@ 通常時はCLOSE

  // ここでタイマーをしかける。今は10msec単位
  ticker.attach_ms( 10, timer0_ISR );

  delay(10);

}

// ---------------------------------------------------------
// 各種種別ボタン選択非選択処理
// ---------------------------------------------------------
void checkIfSelected(uint8_t mask, uint16_t key, int* prevState, bool* isSelected) {
  int val = (key & mask) == 0 ? LOW : HIGH;
  uint16_t setValue;

  int prev_tmp = *prevState;    // 前回ボタン状態保持
  *prevState = val;             // 前回ボタン状態更新

  if (val == HIGH || prev_tmp == LOW) {
    // (ボタン開放 or 押しっぱなし) ... ボタンは Low active
    return;
  }

  // (ボタン押下)
  switch (getState()) {
  case STATE_IDLE:
  case STATE_NETWORK_READY:
  case STATE_STANDALONE_READY:
    if (*isSelected == false) {
      Serial.printf("%d selected\n", mask);
      //状態更新
      //全てをキャンセルしてから選択されたもののみ選択
      allSelectedFlgCancel();
      *isSelected = true;

      setValue = (~(mask << 7) & (key | 0x0F00)); //選択用LEDをまとめて1（消灯）にして、かつ該当ビットのみ0（点灯）にする
      mcp.pinMode(0xFF00); //Output
      mcp.digitalWrite(setValue);

      setState(STATE_STANDALONE_READY);
    } else {
      Serial.printf("%d canceled\n", mask);
      //状態更新
      *isSelected = false;
      selectLedAllOff();
      setAmountLed(0);                                      //@ 分量インジケータも消灯

      setState(STATE_IDLE);
    }
    break;
  case STATE_NETWORK_DISPENSE:
  case STATE_STANDALONE_DISPENSE:
    break;
  }
}

// ---------------------------------------------------------
// 分量インジケータ制御処理
// ---------------------------------------------------------
void setAmountLed(uint16_t amount) {
  if(amount != 0){
    amount -= 1;                                            //@ beta2 エキスパンダ変更対応
  } else {
    amount = 7;
  }
  uint16_t key = mcp.digitalRead();
  uint16_t setValue = ((amount & B00000111) << 12) | (key & 0x8FFF);
  mcp.digitalWrite(setValue);
}

// ---------------------------------------------------------
// ロータリーエンコーダ設定値取得処理
// return 1～7
// ---------------------------------------------------------
int readRotaryEncoder() {
  return g_encValue;
}

// ---------------------------------------------------------
// ロータリーエンコーダ制御処理
// ---------------------------------------------------------
void updateRotaryEncoder() {
  int val = mcp.digitalRead();
  int A = val & MASK_ROTARY_A;
  int B = val & MASK_ROTARY_B;
  int newEnc = (A == 0 ? 2 : 0) | (B == 0 ? 1 : 0);

  if (0 == g_prevEnc) {
    if (2 == newEnc) { // up
      g_encRawValue++;
    } else if (1 == newEnc) { // down
      g_encRawValue--;
    }
  } else if (1 == g_prevEnc) {
    if (0 == newEnc) { // up
      g_encRawValue++;
    } else if (3 == newEnc) { // down
      g_encRawValue--;
    }
  } else if (2 == g_prevEnc) {
    if (3 == newEnc) { // up
      g_encRawValue++;
    } else if (0 == newEnc) { // down
      g_encRawValue--;
    }
  } else if (3 == g_prevEnc) {
    if (1 == newEnc) { // up
      g_encRawValue++;
    } else if (2 == newEnc) { // down
      g_encRawValue--;
    }
  }
  g_prevEnc = newEnc;
  if (g_encRawValue > 7 * COEFF_ENCVALUE) {
    g_encRawValue = 7 * COEFF_ENCVALUE;
  } else if (g_encRawValue < 1 * COEFF_ENCVALUE) {
    g_encRawValue = 1 * COEFF_ENCVALUE;
  }

  g_encValue = g_encRawValue / COEFF_ENCVALUE;
}

// ---------------------------------------------------------
// 分量設定処理
// 単位：ml
// ---------------------------------------------------------
int getAmountforStandAlone() {
  int amount;
  int enc = readRotaryEncoder();

  if (enc == 0) {
    amount = 0;
  } else if (enc == 1) { //小１
    amount = 5;
  } else if (enc == 2) { //小２
    amount = 10;
  } else if (enc == 3) { //大１
    amount = 15;
  } else if (enc == 4) { //大２
    amount = 30;
  } else if (enc == 5) { //大３
    amount = 45;
  } else if (enc == 6) { //大４
    amount = 60;
  } else if (enc == 7) { //連続
    amount = -1;
  }

  return amount;
}

// ---------------------------------------------------------
// 選択種別設定処理
// ---------------------------------------------------------
int getSelectedId() {
  int id;
  if (g_isSoySourceSelected) {
    id = 0;
  } else if (g_isMirinSelected) {
    id = 1;
  } else if (g_isSakeSelected) {
    id = 2;
  } else if (g_isVinegarSelected) {
    id = 3;
  }
  return id;
}

// ---------------------------------------------------------
// 手動吐出時の吐出前設定
// ---------------------------------------------------------
void readHWSettings() {
  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    g_amount[i] = 0;                                       // 吐出量クリア
    g_weight[i] = 0;                                       // 吐出重量クリア
  }

  // 選択されている液種の吐出量設定
  int amount = getAmountforStandAlone();
  if (g_isSoySourceSelected) {
    g_amount[0] = amount;
    g_motorCtrlIndex = 0;
  } else if (g_isMirinSelected) {
    g_amount[1] = amount;
    g_motorCtrlIndex = 1;
  } else if (g_isSakeSelected) {
    g_amount[2] = amount;
    g_motorCtrlIndex = 2;
  } else if (g_isVinegarSelected) {
    g_amount[3] = amount;
    g_motorCtrlIndex = 3;
  }

  // 選択されている液種とその残量をコンソールへ
  Serial.printf("SoySource:%d, Mirin:%d, Sake:%d, Vinegar:%d, Amount:%d\n",
                g_isSoySourceSelected, g_isMirinSelected, g_isSakeSelected, g_isVinegarSelected, amount);

  // モーター回転時間とタイマーカウントを設定
  for ( int i = 0; i < NUM_OF_BOTTLES; i++ ) {
    g_motorRotateTime[i] = GARDTIME;                        //@ 固定
  }
}


// ---------------------------------------------------------
// メイン処理
// ---------------------------------------------------------
void loop() {
  // -------------------------------------------------------
  // HTTPリクエスト処理
  // -------------------------------------------------------
  server.handleClient();

  // -------------------------------------------------------
  // SW情報取得
  // 各ボタンの状態を取得して動作
  // ボタン同士の優先度の定義が必要だが、実使用上同時押しはほぼないとして今はそこまではやらない
  // -------------------------------------------------------

  //GPAをread
  uint16_t keyGPA = mcp.digitalRead();

  //醤油選択
  checkIfSelected(MASK_SOYSOURCE, keyGPA, &g_nPrevKeyState_SoySource, &g_isSoySourceSelected);

  //みりん選択
  checkIfSelected(MASK_MIRIN, keyGPA, &g_nPrevKeyState_Mirin, &g_isMirinSelected);

  //酒選択
  checkIfSelected(MASK_SAKE, keyGPA, &g_nPrevKeyState_Sake, &g_isSakeSelected);

  //酢選択
  checkIfSelected(MASK_VINEGAR, keyGPA, &g_nPrevKeyState_Vinegar, &g_isVinegarSelected);

  //分量設定LEDの更新
  //値の更新は timer0_ISR の中で定期的（10msごと）に実行
  if (getState() == STATE_STANDALONE_READY) {
    int enc = readRotaryEncoder();
    setAmountLed(enc);
  }

  // Push Buttonが押された時の処理
  // ボタンが押下状態→離されたときに実行すること
  // チャタリング除去を行わないとひどいことになるので注意
  // 今は暫定版のなんちゃってチャタリング除去を行っておく。
  int valRoofDispense = ((uint8_t)keyGPA & MASK_ROOFDISPENSE) == 0 ? LOW : HIGH;
  int valFrontDispense = ((uint8_t)keyGPA & MASK_FRONTDISPENSE) == 0 ? LOW : HIGH;
  if (getState() == STATE_STANDALONE_READY && readRotaryEncoder() == 7) { //連続吐出    //STANDALONE_READY && 「連続」選択時（7）
    if ( g_nPrevKeyState_RoofDispense == HIGH && valRoofDispense == LOW
         || g_nPrevKeyState_FrontDispense == HIGH && valFrontDispense == LOW ) { // && H→L
      // ---------------------------------------------------
      // 連続吐出開始
      // stopか確認 //STATE_STANDALONE_READYなら停止中のはず
      // ---------------------------------------------------
      startMotor();                                     // 吐出開始
    }
  } else if (getState() == STATE_STANDALONE_DISPENSE && readRotaryEncoder() == 7
             && ( g_nPrevKeyState_RoofDispense == LOW && valRoofDispense == HIGH
                  || g_nPrevKeyState_FrontDispense == LOW && valFrontDispense == HIGH)) {

    // -----------------------------------------------------
    // 連続吐出終了
    // -----------------------------------------------------
    Serial.println("Continuous Dispense Stop");
    selectLedOn(g_maskArray[getSelectedId()]);              // 該当の液種のみ再度点灯
    stopMotor();                                            // 吐出停止
    writeRemainingAmountOnEEPROM();

    //ログ用フラグ設定
    g_Log_StandAlone = true;
    if (g_nPrevKeyState_RoofDispense == LOW) {
      g_Log_RoofDispensePushed = true;
    } else {
      g_Log_RoofDispensePushed = false;
    }
    g_Log_ContinuousDispense = true;

  } else if ( getState() != STATE_IDLE &&  //レシピ転送のケース
              (g_nPrevKeyState_RoofDispense == LOW && valRoofDispense == HIGH
               || g_nPrevKeyState_FrontDispense == LOW && valFrontDispense == HIGH ) ) {
    if (g_startMotor == false ) {
      // ---------------------------------------------------
      // 手動吐出 or レシピ転送吐出 開始
      // ---------------------------------------------------
      //ボタン識別用
      if (g_nPrevKeyState_RoofDispense == LOW) {
        g_Log_RoofDispensePushed = true;
      } else {
        g_Log_RoofDispensePushed = false;
      }

      startMotor();
    } else {
      // ---------------------------------------------------
      // 途中キャンセル
      // ---------------------------------------------------
      Serial.println("Cancelled");
      if (getState() == STATE_NETWORK_DISPENSE) {
        selectLedAllOff();
      } else if (getState() == STATE_STANDALONE_DISPENSE) {
        selectLedOn(g_maskArray[g_motorCtrlIndex]);
      }
      stopMotor();
      writeRemainingAmountOnEEPROM();
    }
  }

  // -------------------------------------------------------
  // 今回SW状態保存
  // -------------------------------------------------------
  g_nPrevKeyState_RoofDispense = valRoofDispense;
  g_nPrevKeyState_FrontDispense = valFrontDispense;

  // -------------------------------------------------------
  // ログ出力設定
  // -------------------------------------------------------
  if (g_Log_NeedToBeSent){
    if(WiFi.status() == WL_CONNECTED) {
      sendDispenseLog();
    }
    g_Log_NeedToBeSent = false;
  }

  // -------------------------------------------------------
  // debug用コマンド解析
  // -------------------------------------------------------
  // シリアルからのコマンド解析
  // 改行文字はCR+LFで行う。LFでコマンドパースを実行する。
  if ( Serial.available() > 0 ) {
    int ch = Serial.read();

    if ( ch >= 0x00 ) {
      if ( ch == 0x0a ) {
        // コマンドをパースする
        Serial.println(g_RcvBuffer);
        ParseCommand(g_RcvBuffer);
        memset(g_RcvBuffer, 0, sizeof(g_RcvBuffer));
        g_RcvBufferWrPtr = 0;

      } else if ( ch == 0x0d ) {

      } else {
        g_RcvBuffer[g_RcvBufferWrPtr] = ch;
        g_RcvBufferWrPtr++;
      }
    }
  }

  delay(10);
}


