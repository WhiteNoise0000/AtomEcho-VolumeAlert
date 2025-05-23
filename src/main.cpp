#include <M5Atom.h>
#include <driver/i2s.h>

#include "SPIFFS.h"
#include "AudioFileSourceID3.h"
#include "AudioFileSourceSPIFFS.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"

// 音圧の閾値（最大32767の範囲で設定／距離15cm・70～80dBで5000前後？）
#define threshold 3000

// 注意音声のファイル名
#define warning "/alert.mp3"

// 遅延時間設定
#define INITIAL_DELAY_MS 2000
#define RE_ALERT_INTERVAL_MS 10000
#define CONTINUOUS_TRIGGER_COUNT 3 // 連続して閾値を超えた場合に警告を出す回数

// I2Sの設定
#define I2S_NUM I2S_NUM_0      // I2Sの番号
#define I2S_BCK 19             // ビットクロックのピン
#define I2S_LRCK 33            // 左右チャンネルのピン
#define I2S_DIN 23             // マイク入力のピン
#define I2S_DOUT 22            // スピーカ出力のピン
#define I2S_SAMPLE_RATE 16000  // サンプルレート（Hz）

#define MODE_MIC 0
#define MODE_SPK 1

// 現在のI2Sモード
int current_i2s_mode = -1; // 初期値は無効なモード

// 音声関連のオブジェクト
AudioGeneratorMP3 *mp3 = nullptr;
AudioFileSourceSPIFFS *file = nullptr;
AudioOutputI2S *out = nullptr;

// 音声再生中かどうかのフラグ
bool playing = false;
// 連続閾値超過カウンター
int continuous_over_threshold_count = 0;

/**
 * マイク or スイッチモード切り替え
 * サンプル実装ベースにESP8266Audio利用前提で修正
 * @see
 * https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Core/Atom/AtomEcho/Arduino/Repeater/Repeater.ino
 */
void InitI2SSpeakerOrMic(int mode) {
  if (mode == current_i2s_mode) {
    return; // モードが同じ場合は何もしない
  }

  // モード切り替え前に、現在のモードに応じたクリーンアップ
  if (current_i2s_mode == MODE_SPK) {
    if (out != nullptr) {
      out->stop();
      delete out;
      out = nullptr;
    }
  }

  // current_i2s_mode が有効なモード（MICまたはSPK）であり、かつ初期状態(-1)でない場合のみアンインストールを試みる
  if (current_i2s_mode == MODE_MIC || current_i2s_mode == MODE_SPK) {
    esp_err_t uninstall_err = i2s_driver_uninstall(I2S_NUM);
    if (uninstall_err != ESP_OK) {
        Serial.printf("Warning: i2s_driver_uninstall failed (อาจจะไม่ใช่ปัญหาใหญ่): %d\n", uninstall_err);
    }
  }

  if (mode == MODE_MIC) {
    esp_err_t err = ESP_OK;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
    };
    err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Error i2s_driver_install: %d\n", err);
    }
    i2s_pin_config_t tx_pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_LRCK,
        .data_out_num = I2S_DOUT, // 使用しないが設定は必要
        .data_in_num = I2S_DIN,
    };
    err = i2s_set_pin(I2S_NUM, &tx_pin_config);
    if (err != ESP_OK) {
        Serial.printf("Error i2s_set_pin (MIC): %d\n", err);
    }
    err = i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (err != ESP_OK) {
        Serial.printf("Error i2s_set_clk (MIC): %d\n", err);
    }
  }
  // スピーカーモード
  else if (mode == MODE_SPK) {
    // スピーカーの初期化
    // i2s_driver_install は AudioOutputI2S のコンストラクタまたは begin で行われる想定
    // ここではピン設定のみ行う (AudioOutputI2Sライブラリの挙動に依存)
    // もし AudioOutputI2S が内部で driver_install を行わない場合は、ここで行う必要がある
    out = new AudioOutputI2S(); // スピーカーモード移行時に常に新しいインスタンスを作成
    out->SetPinout(I2S_BCK, I2S_LRCK, I2S_DOUT);
    out->SetChannels(1);
    out->SetGain(0.8);  // 音量
  }
  current_i2s_mode = mode;
}

void setup() {
  // M5Atomの初期化
  M5.begin(true, false, true);
  SPIFFS.begin();
  M5.dis.drawpix(0, CRGB(0, 128, 0));
  // マイクモード設定
  InitI2SSpeakerOrMic(MODE_MIC);
  delay(INITIAL_DELAY_MS);
}

// マイクバッファ
#define BUFF_SIZE 1024
int16_t dcBias = 0; // DCバイアスはグローバルで保持
// デバッグ用変数
int16_t debug_maxData = 0;
int16_t debug_dcBias = 0;
/**
 * マイク入力から最大音量取得.
 * @see https://qiita.com/tomoto335/items/263b23d9ba156de12857
 * @return 最大音量
 */
int16_t getMaxVol() {
  uint8_t readBuff[BUFF_SIZE];
  int16_t sumData = 0;
  int16_t maxData = 0;
  size_t bytesread;

  // マイクからの平均音圧を取得する
  i2s_zero_dma_buffer(I2S_NUM);
  esp_err_t err = i2s_read(I2S_NUM, (char *)readBuff, BUFF_SIZE, &bytesread, (100 / portTICK_RATE_MS));
  if (err != ESP_OK) {
    Serial.printf("Error in i2s_read: %d\n", err);
    return 0; // エラー時は0を返す
  }
  if (bytesread == 0) {
    Serial.println("i2s_read returned 0 bytes");
    return 0;
  }

  // マイク入力のサンプルbit数(16bit)へ変換しつつ最大／平均値を特定
  for (int i = 0; i < bytesread; i += 2) { // BUFF_SIZEではなく実際に読み込んだbytesreadまで
    if (i + 1 < bytesread) { // 配列外アクセス防止
        int16_t data = readBuff[i] | (readBuff[i + 1] << 8);  // リトルエンディアン
        sumData += data;
        maxData = max(maxData, data);
    }
  }

  // 平均を取ってDCバイアスを補正し音圧とする
  // @see https://qiita.com/tomoto335/items/263b23d9ba156de12857
  const float alpha = 0.98;
  // bytesread が十分な大きさの場合のみ dcBias を更新
  if (bytesread >= BUFF_SIZE / 2) { // 例えばバッファの半分以上のデータがある場合
    int16_t current_avg = sumData / (bytesread / 2);
    dcBias = dcBias * alpha + current_avg * (1 - alpha);
  }
  // dcBias が極端な値にならないようにクリップ (値は調整の余地あり)
  if (dcBias < -5000) dcBias = -5000;
  if (dcBias > 5000) dcBias = 5000;

  debug_maxData = maxData; // デバッグ用
  debug_dcBias = dcBias;   // デバッグ用
  return maxData - dcBias;
}

void loop() {
  // M5Atomの更新
  M5.update();

  // 音声再生中の場合
  if (playing) {
    // 音声再生を続ける
    if (mp3->isRunning()) {
      if (!mp3->loop()) {
        mp3->stop();
        delete mp3; // メモリ解放
        mp3 = nullptr;
        delete file; // メモリ解放
        file = nullptr;
        if (out != nullptr) { // outがMODE_SPKで使われていたら
            // out->stop(); // mp3->stop()で内部的に呼ばれることが多いが念のため
            // delete out; // InitI2SSpeakerOrMic(MODE_MIC)で再生成されるのでここでは不要
            // out = nullptr;
        }
        playing = false;
        // マイクモード再設定
        InitI2SSpeakerOrMic(MODE_MIC);
        M5.dis.drawpix(0, CRGB(0, 128, 0));
        // 10秒は再警告しない
        delay(RE_ALERT_INTERVAL_MS);
      }
    }
    return;
  }

  // 音圧が閾値を超えた場合
  int16_t maxVol = getMaxVol();
  Serial.printf("maxVol: %d (maxData: %d, dcBias: %d, threshold: %d, count: %d)\n", maxVol, debug_maxData, debug_dcBias, threshold, continuous_over_threshold_count);

  if (maxVol > threshold) {
    continuous_over_threshold_count++;
    if (continuous_over_threshold_count >= CONTINUOUS_TRIGGER_COUNT) {
      // 注意音声を再生する
      InitI2SSpeakerOrMic(MODE_SPK);
      M5.dis.drawpix(0, CRGB(128, 0, 0));
      // 注意音声(mp3)
      file = new AudioFileSourceSPIFFS(warning);
      mp3 = new AudioGeneratorMP3();
      mp3->begin(file, out);
      playing = true;
      continuous_over_threshold_count = 0; // 再生後はカウンターリセット
    }
  } else {
    continuous_over_threshold_count = 0; // 閾値を下回ったらカウンターリセット
  }
}
