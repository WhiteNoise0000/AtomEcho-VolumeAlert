#include <M5Atom.h>
#include <driver/i2s.h>

#include "AudioFileSourceID3.h"
#include "AudioFileSourceSPIFFS.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include "SPIFFS.h"

// 音圧の閾値（最大32767の範囲で設定）
#define threshold 23000

// 注意音声のファイル名
#define warning "/alert.mp3"

// I2Sの設定
#define I2S_NUM I2S_NUM_0      // I2Sの番号
#define I2S_BCK 19             // ビットクロックのピン
#define I2S_LRCK 33            // 左右チャンネルのピン
#define I2S_DIN 23             // マイク入力のピン
#define I2S_DOUT 22            // スピーカ出力のピン
#define I2S_SAMPLE_RATE 16000  // サンプルレート（Hz）

#define MODE_MIC 0
#define MODE_SPK 1

// 音声関連のオブジェクト
AudioGeneratorMP3 *mp3;
AudioFileSourceSPIFFS *file;
AudioOutputI2S *out;

// 音声再生中かどうかのフラグ
bool playing = false;

/**
 * マイク or スイッチモード切り替え
 * サンプル実装ベースにESP8266Audio利用前提で修正
 * @see
 * https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Core/Atom/AtomEcho/Arduino/Repeater/Repeater.ino
 */
void InitI2SSpeakerOrMic(int mode) {
  if (out != nullptr) {
    out->stop();
    out = nullptr;
  }
  i2s_driver_uninstall(I2S_NUM);

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
    err += i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_LRCK,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_DIN,
    };
    err += i2s_set_pin(I2S_NUM, &tx_pin_config);
    err += i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT,
                       I2S_CHANNEL_MONO);
  }

  // スピーカーモード
  else {
    // スピーカーの初期化
    out = new AudioOutputI2S();
    out->SetPinout(I2S_BCK, I2S_LRCK, I2S_DOUT);
    out->SetChannels(1);
    out->SetGain(0.8);  // 音量
  }
}

void setup() {
  // M5Atomの初期化
  M5.begin(true, false, true);
  SPIFFS.begin();
  M5.dis.drawpix(0, CRGB(0, 128, 0));
  // マイクモード設定
  InitI2SSpeakerOrMic(MODE_MIC);
  delay(2000);
}

// マイクバッファ
#define BUFF_SIZE 1024
uint8_t readBuff[BUFF_SIZE];
int16_t sumData, maxData, maxVol, dcBias = 0;
size_t bytesread, byteswritten;
void loop() {
  // M5Atomの更新
  M5.update();

  // 音声再生中の場合
  if (playing) {
    // 音声再生を続ける
    if (mp3->isRunning()) {
      if (!mp3->loop()) {
        mp3->stop();
        playing = false;
        // 10秒は再警告しない
        delay(10000);
        // マイクモード再設定
        InitI2SSpeakerOrMic(MODE_MIC);
        M5.dis.drawpix(0, CRGB(0, 128, 0));
      }
    }
    return;
  }

  // マイクからの平均音圧を取得する
  i2s_read(I2S_NUM, (char *)readBuff, BUFF_SIZE, &bytesread,
           (100 / portTICK_RATE_MS));
  // マイク入力のサンプルbit数(16bit)へ変換しつつ最大／平均値を特定
  sumData = 0;
  maxData = 0;
  for (int i = 0; i < BUFF_SIZE; i += 2) {
    int16_t data = readBuff[i] | (readBuff[i + 1] << 8);  // リトルエンディアン
    sumData += data;
    maxData = max(maxData, data);
  }

  // 平均を取ってDCバイアスを補正し音圧とする
  // @see https://qiita.com/tomoto335/items/263b23d9ba156de12857
  const float alpha = 0.98;
  dcBias = dcBias * alpha + (sumData / (BUFF_SIZE / 2)) * (1 - alpha);
  maxVol = maxData - dcBias;
  Serial.println(maxVol);

  // 音圧が閾値を超えた場合
  if (maxVol > threshold) {
    // 注意音声を再生する
    InitI2SSpeakerOrMic(MODE_SPK);
    M5.dis.drawpix(0, CRGB(128, 0, 0));
    // 注意音声(mp3)
    file = new AudioFileSourceSPIFFS(warning);
    mp3 = new AudioGeneratorMP3();
    mp3->begin(file, out);
    playing = true;
  }
}
