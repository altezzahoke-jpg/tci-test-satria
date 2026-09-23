#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "esp_idf_version.h"
#include "esp_netif.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define ADC_ATTEN_TARGET ADC_ATTEN_DB_12
#else
#define ADC_ATTEN_TARGET ADC_ATTEN_DB_11
#endif

// -----------------------------------------------------------------------------
// KONFIGURASI HARDWARE & PIN ESP32-S3
// -----------------------------------------------------------------------------
#define PIN_PULSER       GPIO_NUM_4
#define PIN_TCI          GPIO_NUM_17
#define PIN_BUTTON       GPIO_NUM_6
#define PIN_BTN_WIFI     GPIO_NUM_7
#define PIN_TX_TELEMETRY GPIO_NUM_16
#define PIN_RX_TELEMETRY GPIO_NUM_15
#define PIN_TACH_OUT     GPIO_NUM_8

#define TCI_COIL_CHARGE() REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << PIN_TCI)) 
#define TCI_COIL_SPARK()  REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << PIN_TCI)) 

#define ADC_CHAN_TPS     ADC_CHANNEL_0 
#define ADC_CHAN_BATT    ADC_CHANNEL_1 
#define ADC_CHAN_TEMP    ADC_CHANNEL_2 

#define WDT_TIMEOUT_SECONDS 2
#define AP_SSID "TCI_SatriaFU_Pro"
#define AP_PASS "12345678" 
#define API_SECRET_TOKEN "satria123"

portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE webMux = portMUX_INITIALIZER_UNLOCKED;

typedef enum { 
  STATE_STOPPED = 0, 
  STATE_CRANKING, 
  STATE_RUNNING, 
  STATE_LIMP_HOME 
} EngineState;

volatile EngineState currentEngineState = STATE_STOPPED;

#pragma pack(push, 1)
typedef struct {
  uint16_t header;     
  uint16_t rpm;       
  uint8_t  tps;        
  int16_t  degree10;  
  int16_t  temp10;    
  uint16_t battVolt10;
  uint8_t  mode;       
  uint8_t  fuel;      
  uint8_t  state;      
  uint16_t crc16;      
} TelemetryData;

typedef struct {
  uint16_t header;     
  uint8_t requestedMode;
  uint16_t crc16;      
} CommandData;
#pragma pack(pop)

gptimer_handle_t timerDelay = NULL;
gptimer_handle_t timerDwell = NULL;
gptimer_handle_t timerSafetyDwell = NULL;

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
httpd_handle_t server = NULL;

const int16_t ROTOR_PULSER_DEGREES_10 = 350; 
const uint32_t BASE_DWELL_US          = 2500;
const uint32_t MAX_SAFETY_DWELL_US    = 3500; 

const uint16_t MAX_RPM_LIMIT    = 12500; 
const int16_t  MAX_TEMP_LIMIT10 = 1100;  

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
#define MAX_CUSTOM_SLOTS 5

static char web_post_buffer[4096]; 

typedef enum { MODE_DAILY = 0, MODE_RACING, MODE_EXTREME, MODE_CUSTOM } ModePengapian;
typedef enum { BBM_PERTALITE = 0, BBM_PERTAMAX } JenisBBM;

volatile ModePengapian currentMode = MODE_DAILY;
volatile JenisBBM currentFuel      = BBM_PERTALITE;
volatile uint8_t currentCustomSlot = 0;

volatile uint16_t currentBatteryVoltage10 = 126;
volatile int16_t  currentEngineTemp10     = 300;
volatile uint8_t  currentTPS              = 0;  
volatile uint16_t currentRPM              = 0;
volatile int16_t  currentDegree10         = 100;
volatile bool     sensorFaultStatus       = false;

volatile bool isTuningActive = false;
volatile bool wirelessActive = false; 

DRAM_ATTR const uint16_t rpmAxis[NUM_RPM_POINTS] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000 };
DRAM_ATTR const uint8_t  tpsAxis[NUM_TPS_POINTS] = { 0, 25, 50, 75, 100 };

// Map Base Daily (Standar/Aman)
DRAM_ATTR const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};

// Map Racing (Kurva lebih agresif di putaran menengah)
DRAM_ATTR const int16_t mapRacing3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {160, 170, 180, 190, 200},
  {210, 230, 250, 260, 270}, {250, 270, 290, 300, 310}, {280, 300, 320, 330, 340},
  {310, 330, 350, 360, 370}, {330, 350, 370, 380, 390}, {340, 360, 380, 390, 390},
  {340, 360, 380, 390, 390}, {330, 350, 370, 380, 380}, {310, 330, 350, 360, 360},
  {290, 310, 330, 340, 340}, {270, 290, 310, 320, 320}, {260, 270, 280, 280, 280}
};

// Map Extreme (Kurva sangat agresif, butuh oktan tinggi)
DRAM_ATTR const int16_t mapExtreme3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {180, 190, 200, 210, 220},
  {240, 260, 280, 290, 300}, {280, 300, 320, 330, 340}, {310, 330, 350, 360, 370},
  {340, 360, 380, 390, 400}, {360, 380, 400, 410, 420}, {370, 390, 410, 420, 420},
  {370, 390, 410, 420, 420}, {360, 380, 400, 410, 410}, {340, 360, 380, 390, 390},
  {320, 340, 360, 370, 370}, {300, 320, 340, 350, 350}, {280, 290, 300, 300, 300}
};

DRAM_ATTR int16_t mapCustomSlots[MAX_CUSTOM_SLOTS][NUM_RPM_POINTS][NUM_TPS_POINTS];
DRAM_ATTR int16_t activeCustomMapBuffer[2][NUM_RPM_POINTS][NUM_TPS_POINTS];
volatile uint8_t activeMapIndex = 0;

volatile uint64_t last_pulse_time = 0;
volatile uint64_t pulse_interval  = 0;
volatile uint64_t prev_interval   = 0; 

volatile uint32_t dwellUsGlobal = 2500; 

static inline bool isEngineStopped(void) {
  portENTER_CRITICAL(&isrMux);
  EngineState st = currentEngineState;
  uint16_t rpm = currentRPM;
  portEXIT_CRITICAL(&isrMux);
  return (st == STATE_STOPPED && rpm == 0);
}

void logDTC(const char* errorCode) {
  nvs_handle_t my_handle;
  if (nvs_open("ecu_dtc", NVS_READWRITE, &my_handle) == ESP_OK) {
    nvs_set_str(my_handle, "last_dtc", errorCode);
    nvs_commit(my_handle);
    nvs_close(my_handle);
  }
}

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void updateActiveMapBuffer(void) {
  uint8_t slot = currentCustomSlot;
  if (slot >= MAX_CUSTOM_SLOTS) slot = 0;
  uint8_t nextBufferIndex = 1 - activeMapIndex;
  memcpy(activeCustomMapBuffer[nextBufferIndex], mapCustomSlots[slot], sizeof(activeCustomMapBuffer[0]));
  portENTER_CRITICAL(&isrMux);
  activeMapIndex = nextBufferIndex;
  portEXIT_CRITICAL(&isrMux);
}

esp_err_t saveCustomMapToNVS(uint8_t slot) {
  if (slot >= MAX_CUSTOM_SLOTS) return ESP_ERR_INVALID_ARG;
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("ecu_maps", NVS_READWRITE, &my_handle);
  if (err == ESP_OK) {
    char keyMap[16];
    snprintf(keyMap, sizeof(keyMap), "map%d", slot);
    err = nvs_set_blob(my_handle, keyMap, &mapCustomSlots[slot], sizeof(mapCustomSlots[slot]));
    if (err == ESP_OK) nvs_commit(my_handle);
    nvs_close(my_handle);
  }
  return err;
}

void loadCustomMap(void) {
  nvs_handle_t my_handle;
  if (nvs_open("ecu_maps", NVS_READWRITE, &my_handle) == ESP_OK) {
    for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
      char keyMap[16];
      snprintf(keyMap, sizeof(keyMap), "map%d", i);
      size_t required_size = sizeof(mapCustomSlots[i]);
      if (nvs_get_blob(my_handle, keyMap, &mapCustomSlots[i], &required_size) != ESP_OK) {
        memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
      }
    }
    nvs_close(my_handle);
  } else {
    for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
      memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
    }
  }
  updateActiveMapBuffer();
}

static inline void IRAM_ATTR fast_set_gptimer_alarm(gptimer_handle_t timer, uint32_t delay_us) {
  gptimer_stop(timer); 
  gptimer_alarm_config_t alarm_config = {
    .alarm_count = delay_us,
    .reload_count = 0,
    .flags.auto_reload_on_alarm = false
  };
  gptimer_set_raw_count(timer, 0);
  gptimer_set_alarm_action(timer, &alarm_config);
  gptimer_start(timer);
}

static bool IRAM_ATTR onSafetyDwellTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  TCI_COIL_SPARK(); 
  return false;
}

static bool IRAM_ATTR onDwellTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  TCI_COIL_SPARK(); 
  gptimer_stop(timerSafetyDwell);
  return false;
}

static bool IRAM_ATTR onDelayTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  TCI_COIL_CHARGE(); 
  portENTER_CRITICAL_ISR(&isrMux);
  uint32_t currentDwell = dwellUsGlobal;
  portEXIT_CRITICAL_ISR(&isrMux);
  
  fast_set_gptimer_alarm(timerSafetyDwell, MAX_SAFETY_DWELL_US);
  fast_set_gptimer_alarm(timerDwell, currentDwell);
  return false;
}

int16_t IRAM_ATTR get3DAdvanceDegreeFixed(uint16_t rpm, uint8_t tps, ModePengapian mode) {
  if (rpm < rpmAxis[0]) rpm = rpmAxis[0];
  if (rpm > rpmAxis[NUM_RPM_POINTS - 1]) rpm = rpmAxis[NUM_RPM_POINTS - 1];
  if (tps > 100) tps = 100;

  uint8_t r0 = 0, t0 = 0;
  while (r0 < NUM_RPM_POINTS - 2 && rpm >= rpmAxis[r0 + 1]) r0++;
  while (t0 < NUM_TPS_POINTS - 2 && tps >= tpsAxis[t0 + 1]) t0++;
  uint8_t r1 = r0 + 1, t1 = t0 + 1;

  int16_t q11, q21, q12, q22;
  
  if (mode == MODE_DAILY) {
    q11 = mapDaily3DBase[r0][t0]; q21 = mapDaily3DBase[r1][t0]; 
    q12 = mapDaily3DBase[r0][t1]; q22 = mapDaily3DBase[r1][t1];
  } else if (mode == MODE_RACING) {
    q11 = mapRacing3DBase[r0][t0]; q21 = mapRacing3DBase[r1][t0]; 
    q12 = mapRacing3DBase[r0][t1]; q22 = mapRacing3DBase[r1][t1];
  } else if (mode == MODE_EXTREME) {
    q11 = mapExtreme3DBase[r0][t0]; q21 = mapExtreme3DBase[r1][t0]; 
    q12 = mapExtreme3DBase[r0][t1]; q22 = mapExtreme3DBase[r1][t1];
  } else {
    uint8_t bufIdx = activeMapIndex;
    q11 = activeCustomMapBuffer[bufIdx][r0][t0]; 
    q21 = activeCustomMapBuffer[bufIdx][r1][t0]; 
    q12 = activeCustomMapBuffer[bufIdx][r0][t1]; 
    q22 = activeCustomMapBuffer[bufIdx][r1][t1];
  }

  int32_t rF = ((int32_t)(rpm - rpmAxis[r0]) * 1024) / (rpmAxis[r1] - rpmAxis[r0]);
  int32_t tF = ((int32_t)(tps - tpsAxis[t0]) * 1024) / (tpsAxis[t1] - tpsAxis[t0]);
  int32_t R1 = q11 + ((q21 - q11) * rF) / 1024;
  int32_t R2 = q12 + ((q22 - q12) * rF) / 1024;
  int16_t finalAdv = (int16_t)(R1 + ((R2 - R1) * tF) / 1024);

  int16_t temp = currentEngineTemp10;
  if (temp > MAX_TEMP_LIMIT10) { 
    finalAdv -= 20; 
    if (finalAdv < 100) finalAdv = 100; 
  }

  if (finalAdv > ROTOR_PULSER_DEGREES_10) finalAdv = ROTOR_PULSER_DEGREES_10;
  return finalAdv;
}

static void IRAM_ATTR pulserISR(void* arg) {
  uint64_t now = esp_timer_get_time(); 
  
  portENTER_CRITICAL_ISR(&isrMux);
  uint64_t last = last_pulse_time;
  uint64_t p_interval = pulse_interval;
  uint64_t local_prev = prev_interval;
  uint8_t localTPS = currentTPS; 
  uint16_t localBatt10 = currentBatteryVoltage10;
  ModePengapian localMode = currentMode; 
  bool isTuning = isTuningActive; 
  bool isFault = sensorFaultStatus;
  portEXIT_CRITICAL_ISR(&isrMux);

  uint64_t interval = now - last;
  
  if (interval < 2500 && last != 0) return; 
  if (p_interval > 0) {
    uint64_t min_valid_interval = (p_interval * 30) / 100;
    if (interval < min_valid_interval) return; 
  }
  
  int64_t max_delta_pct = 30;
  int64_t interval_delta = (int64_t)interval - (int64_t)local_prev;
  int64_t max_delta = (int64_t)(local_prev * max_delta_pct / 100);  
  if (interval_delta > max_delta) interval_delta = max_delta;
  if (interval_delta < -max_delta) interval_delta = -max_delta;

  uint64_t predicted_interval = (local_prev > 0) ? ((interval * 2 + local_prev) / 3 + interval_delta) : interval;
  
  uint16_t rpm = 0;
  if (predicted_interval > 0) rpm = (uint16_t)(60000000ULL / predicted_interval);
  
  EngineState newState = (isFault) ? STATE_LIMP_HOME : ((rpm < 600) ? STATE_CRANKING : STATE_RUNNING);

  portENTER_CRITICAL_ISR(&isrMux); 
  pulse_interval = interval;
  last_pulse_time = now;
  prev_interval = interval;
  currentRPM = rpm;
  currentEngineState = newState;
  portEXIT_CRITICAL_ISR(&isrMux);

  if (isTuning || rpm > MAX_RPM_LIMIT || (newState == STATE_LIMP_HOME && rpm > 4000) || predicted_interval == 0) {
    TCI_COIL_SPARK();
    return;
  }

  int16_t adv10; 
  uint32_t targetDwellUs;
  
  if (newState == STATE_LIMP_HOME) {
    adv10 = 100; 
    targetDwellUs = 2500; 
  } else if (newState == STATE_CRANKING) {
    adv10 = 50;  
    targetDwellUs = 3000;
  } else {
    adv10 = get3DAdvanceDegreeFixed(rpm, localTPS, localMode);
    if (localBatt10 <= 100) targetDwellUs = 3500;
    else if (localBatt10 >= 150) targetDwellUs = 2000;
    else targetDwellUs = 3500 - ((uint32_t)(localBatt10 - 100) * 1500) / 50;
  }
  
  currentDegree10 = adv10; 

  uint32_t maxAllowedDwell = (uint32_t)(predicted_interval * 7 / 10);
  if (targetDwellUs > maxAllowedDwell) targetDwellUs = maxAllowedDwell;
  if (targetDwellUs > MAX_SAFETY_DWELL_US) targetDwellUs = MAX_SAFETY_DWELL_US;
  if (targetDwellUs < 800) targetDwellUs = 800; 

  int16_t sparkDegFromPulser = ROTOR_PULSER_DEGREES_10 - adv10;
  if (sparkDegFromPulser < 0) sparkDegFromPulser = 0;
  uint32_t sparkDelayUs = (uint32_t)(((uint64_t)sparkDegFromPulser * predicted_interval) / 3600ULL);
  
  if (sparkDelayUs >= predicted_interval) {
    TCI_COIL_SPARK(); 
    return;
  }

  uint32_t finalChargeDelayUs;
  uint32_t finalDwellUs;

  if (sparkDelayUs > targetDwellUs) {
    finalChargeDelayUs = sparkDelayUs - targetDwellUs;
    finalDwellUs = targetDwellUs;
  } else {
    finalChargeDelayUs = 5; 
    finalDwellUs = (sparkDelayUs > 10) ? (sparkDelayUs - 5) : 5;
  }

  dwellUsGlobal = finalDwellUs; 
  fast_set_gptimer_alarm(timerDelay, finalChargeDelayUs);
}

long mapRange(long x, long in_min, long in_max, long out_min, long out_max) {
  long result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min) result = out_min;
  if (result > out_max) result = out_max;
  return result;
}

void codeTaskSensor(void * parameter) {
  esp_task_wdt_add(NULL); 
  static uint32_t emaTpsInt = 0, emaBatt = 0, emaTemp = 0;
  static uint8_t faultCounter = 0;
  static bool dtc_logged = false;
  
  for(;;) {
    esp_task_wdt_reset(); 
    
    int rawTps = 0, rawBatt = 0, rawTemp = 0;
    int mvTps = 0, mvBatt = 0, mvTemp = 0;

    adc_oneshot_read(adc1_handle, ADC_CHAN_TPS, &rawTps);
    adc_oneshot_read(adc1_handle, ADC_CHAN_BATT, &rawBatt);
    adc_oneshot_read(adc1_handle, ADC_CHAN_TEMP, &rawTemp);

    if (adc1_cali_handle) {
      adc_cali_raw_to_voltage(adc1_cali_handle, rawTps, &mvTps);
      adc_cali_raw_to_voltage(adc1_cali_handle, rawBatt, &mvBatt);
      adc_cali_raw_to_voltage(adc1_cali_handle, rawTemp, &mvTemp);
    } else {
      mvTps = rawTps; mvBatt = rawBatt; mvTemp = rawTemp;
    }

    bool rawFault = (rawBatt < 100 || rawBatt > 4050 || rawTps > 4050 || rawTemp < 50 || rawTemp > 4050);
    if (rawFault) {
      if (faultCounter < 10) faultCounter++;
    } else {
      if (faultCounter > 0) faultCounter--;
    }
    bool isFaulty = (faultCounter >= 10);

    if (isFaulty && !dtc_logged) {
      logDTC("P0122_SENSOR_MULTIPLE_FAULT");
      dtc_logged = true;
    } else if (!isFaulty && dtc_logged) {
      dtc_logged = false; 
    }
    
    emaTpsInt = (emaTpsInt == 0) ? (mvTps << 2) : (emaTpsInt - (emaTpsInt >> 2) + mvTps);
    emaBatt   = (emaBatt == 0)   ? (mvBatt << 2) : (emaBatt - (emaBatt >> 2) + mvBatt);
    emaTemp   = (emaTemp == 0)   ? (mvTemp << 2) : (emaTemp - (emaTemp >> 2) + mvTemp);

    uint16_t cT    = emaTpsInt >> 2;
    uint16_t cB    = emaBatt >> 2;
    uint16_t cTemp = emaTemp >> 2;

    uint8_t tpsP    = (uint8_t)mapRange(cT, 450, 2800, 0, 100); 
    uint16_t battV  = (uint16_t)mapRange(cB, 0, 3100, 0, 160);
    int16_t tempC   = (int16_t)mapRange(cTemp, 300, 2800, -200, 1500);

    portENTER_CRITICAL(&isrMux);
    currentTPS = tpsP; 
    currentBatteryVoltage10 = battV; 
    currentEngineTemp10 = tempC; 
    sensorFaultStatus = isFaulty; 
    
    uint64_t now = esp_timer_get_time();
    if ((now - last_pulse_time) > 200000ULL && currentRPM != 0) {
        currentRPM = 0;
        currentEngineState = STATE_STOPPED;
    }
    portEXIT_CRITICAL(&isrMux);
    
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void codeTaskTachOutput(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint32_t lastFreqHz = 0;

  for (;;) {
    esp_task_wdt_reset();
    uint16_t rpm; 
    portENTER_CRITICAL(&isrMux); 
    rpm = currentRPM; 
    portEXIT_CRITICAL(&isrMux);
    
    if (rpm > 400) {
      uint32_t freqHz = rpm / 60; 
      if (freqHz < 1) freqHz = 1;
      
      if (freqHz != lastFreqHz) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freqHz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512); 
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        lastFreqHz = freqHz;
      }
    } else {
      if (lastFreqHz != 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        lastFreqHz = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void codeTaskTelemetry(void * parameter) {
  esp_task_wdt_add(NULL);
  for(;;) {
    esp_task_wdt_reset();
    TelemetryData pkt;
    
    portENTER_CRITICAL(&isrMux);
    pkt.header = 0xAA55; 
    pkt.rpm = currentRPM; 
    pkt.tps = currentTPS; 
    pkt.degree10 = currentDegree10;
    pkt.temp10 = currentEngineTemp10; 
    pkt.battVolt10 = currentBatteryVoltage10;
    pkt.mode = currentMode; 
    pkt.fuel = currentFuel; 
    pkt.state = currentEngineState;
    portEXIT_CRITICAL(&isrMux);

    pkt.crc16 = calculateCRC16((uint8_t*)&pkt, sizeof(TelemetryData) - sizeof(uint16_t));
    uart_write_bytes(UART_NUM_2, (const char*)&pkt, sizeof(TelemetryData));
    vTaskDelay(pdMS_TO_TICKS(40)); 
  }
}

void codeTaskCommandListener(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint8_t cmdBuf[sizeof(CommandData)]; 
  static size_t cmdIdx = 0;
  for(;;) {
    esp_task_wdt_reset();
    uint8_t c;
    while (uart_read_bytes(UART_NUM_2, &c, 1, 0) > 0) {
      if (cmdIdx == 0 && c != 0x55) continue;
      if (cmdIdx == 1 && c != 0xCC) { cmdIdx = 0; continue; }
      cmdBuf[cmdIdx++] = c;
      if (cmdIdx >= sizeof(CommandData)) {
        cmdIdx = 0; 
        CommandData *cmd = (CommandData*)cmdBuf;
        if (cmd->crc16 == calculateCRC16(cmdBuf, sizeof(CommandData) - sizeof(uint16_t))) {
          if (isEngineStopped() && cmd->requestedMode <= MODE_CUSTOM) {
            portENTER_CRITICAL(&isrMux);
            currentMode = (ModePengapian)cmd->requestedMode;
            portEXIT_CRITICAL(&isrMux);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// -----------------------------------------------------------------------------
// WEBSERVER HANDLERS WITH STATIC MEMORY
// -----------------------------------------------------------------------------

esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  // Pastikan Anda menempelkan source HTML Web UI Anda di baris ini
  httpd_resp_send_chunk(req, "<!DOCTYPE html><html><head>... (Potongan HTML Web UI) ... </html>", HTTPD_RESP_USE_STRLEN);
  httpd_resp_send_chunk(req, NULL, 0); 
  return ESP_OK;
}

esp_err_t data_get_handler(httpd_req_t *req) {
  char json_resp[128];
  portENTER_CRITICAL(&isrMux);
  uint16_t r = currentRPM;
  uint8_t  t = currentTPS;
  int16_t  temp = currentEngineTemp10;
  uint16_t b = currentBatteryVoltage10;
  portEXIT_CRITICAL(&isrMux);

  snprintf(json_resp, sizeof(json_resp), "{\"rpm\":%d,\"tps\":%d,\"temp\":%d,\"batt\":%d}", r, t, temp, b);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t setmode_get_handler(httpd_req_t *req) {
  if (!isEngineStopped()) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "GAGAL: Mesin harus mati untuk mengubah mode!");
    return ESP_OK;
  }
  char buf[64];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char tokenParam[32];
    if (httpd_query_key_value(buf, "token", tokenParam, sizeof(tokenParam)) != ESP_OK || strcmp(tokenParam, API_SECRET_TOKEN) != 0) {
      httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "AKSES DITOLAK: Token Keamanan Tidak Valid!");
      return ESP_OK;
    }
    char param[16];
    if (httpd_query_key_value(buf, "mode", param, sizeof(param)) == ESP_OK) {
      int m = atoi(param);
      portENTER_CRITICAL(&isrMux);
      if (m <= MODE_CUSTOM) currentMode = (ModePengapian)m;
      portEXIT_CRITICAL(&isrMux);
    }
  }
  const char* modeStr[] = {"Mode Daily Aktif", "Mode Racing Aktif", "Mode Extreme Aktif", "Mode Custom Aktif"};
  uint8_t mIdx = currentMode;
  if (mIdx > 3) mIdx = 0;
  httpd_resp_send(req, modeStr[mIdx], HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t getmap_get_handler(httpd_req_t *req) {
  char buf[32];
  int idx = 0;
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char param[16];
    if (httpd_query_key_value(buf, "idx", param, sizeof(param)) == ESP_OK) {
      idx = atoi(param);
      if (idx >= MAX_CUSTOM_SLOTS) idx = 0;
    }
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send_chunk(req, "[", 1);
  char cellBuf[32];
  for (int r = 0; r < NUM_RPM_POINTS; r++) {
    httpd_resp_send_chunk(req, "[", 1);
    for (int c = 0; c < NUM_TPS_POINTS; c++) {
      snprintf(cellBuf, sizeof(cellBuf), "%d%s", mapCustomSlots[idx][r][c], (c < NUM_TPS_POINTS - 1) ? "," : "");
      httpd_resp_send_chunk(req, cellBuf, HTTPD_RESP_USE_STRLEN);
    }
    snprintf(cellBuf, sizeof(cellBuf), "]%s", (r < NUM_RPM_POINTS - 1) ? "," : "");
    httpd_resp_send_chunk(req, cellBuf, HTTPD_RESP_USE_STRLEN);
  }
  httpd_resp_send_chunk(req, "]", 1);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

esp_err_t savemap_post_handler(httpd_req_t *req) {
  if (!isEngineStopped()) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "AKSES DITOLAK: Mesin harus mati untuk menyimpan Map!");
    return ESP_OK;
  }

  portENTER_CRITICAL(&webMux);
  memset(web_post_buffer, 0, sizeof(web_post_buffer));

  int ret = httpd_req_recv(req, web_post_buffer, sizeof(web_post_buffer) - 1);
  if (ret <= 0) {
    portEXIT_CRITICAL(&webMux);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  
  char tokenVal[32];
  if (httpd_query_key_value(web_post_buffer, "token", tokenVal, sizeof(tokenVal)) != ESP_OK || strcmp(tokenVal, API_SECRET_TOKEN) != 0) {
    portEXIT_CRITICAL(&webMux);
    httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "AKSES DITOLAK: Token Keamanan Tidak Valid!");
    return ESP_OK;
  }

  int idx = 0;
  char valStr[16];
  if (httpd_query_key_value(web_post_buffer, "idx", valStr, sizeof(valStr)) == ESP_OK) {
    idx = atoi(valStr);
    if (idx >= MAX_CUSTOM_SLOTS) idx = 0;
  }
  
  char key[16];
  for (int r = 0; r < NUM_RPM_POINTS; r++) {
    for (int c = 0; c < NUM_TPS_POINTS; c++) {
      snprintf(key, sizeof(key), "v_%d_%d", r, c);
      if (httpd_query_key_value(web_post_buffer, key, valStr, sizeof(valStr)) == ESP_OK) {
        if (strlen(valStr) > 0) {
          char *endptr;
          long parsed = strtol(valStr, &endptr, 10);
          if (*endptr == '\0') {
            if (parsed < 0) parsed = 0;
            if (parsed > ROTOR_PULSER_DEGREES_10) parsed = ROTOR_PULSER_DEGREES_10;
            mapCustomSlots[idx][r][c] = (int16_t)parsed;
          }
        }
      }
    }
  }
  portEXIT_CRITICAL(&webMux);

  updateActiveMapBuffer();
  saveCustomMapToNVS(idx);

  httpd_resp_send(req, "Map Berhasil Disimpan ke NVS Flash!", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

void setWirelessState(bool enable) {
  portENTER_CRITICAL(&isrMux); 
  wirelessActive = enable; 
  portEXIT_CRITICAL(&isrMux);
  
  if (enable) {
    esp_wifi_start();
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 10;
    
    if (server == NULL && httpd_start(&server, &config) == ESP_OK) {
      httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
      httpd_uri_t data_uri = { .uri = "/data", .method = HTTP_GET, .handler = data_get_handler, .user_ctx = NULL };
      httpd_uri_t setmode_uri = { .uri = "/setmode", .method = HTTP_GET, .handler = setmode_get_handler, .user_ctx = NULL };
      httpd_uri_t getmap_uri = { .uri = "/getmap", .method = HTTP_GET, .handler = getmap_get_handler, .user_ctx = NULL };
      httpd_uri_t savemap_uri = { .uri = "/savemap", .method = HTTP_POST, .handler = savemap_post_handler, .user_ctx = NULL };

      httpd_register_uri_handler(server, &root);
      httpd_register_uri_handler(server, &data_uri);
      httpd_register_uri_handler(server, &setmode_uri);
      httpd_register_uri_handler(server, &getmap_uri);
      httpd_register_uri_handler(server, &savemap_uri);
    }
  } else {
    if (server) { 
      httpd_stop(server); 
      server = NULL; 
    }
    esp_wifi_stop();
  }
}

void codeTaskNetwork(void * parameter) {
  esp_task_wdt_add(NULL);
  
  setWirelessState(false);
  uint32_t lastModeBtn = 0, lastWifiBtn = 0;
  
  for (;;) {
    esp_task_wdt_reset(); 
    uint32_t now = esp_timer_get_time() / 1000;
    
    if (gpio_get_level(PIN_BUTTON) == 0 && (now - lastModeBtn > 300)) {
      if (isEngineStopped()) {
        portENTER_CRITICAL(&isrMux);
        currentMode = (ModePengapian)((currentMode + 1) % 4); 
        portEXIT_CRITICAL(&isrMux); 
      }
      lastModeBtn = now;
    }
    
    if (gpio_get_level(PIN_BTN_WIFI) == 0 && (now - lastWifiBtn > 300)) {
      portENTER_CRITICAL(&isrMux); 
      bool nextWifiState = !wirelessActive; 
      portEXIT_CRITICAL(&isrMux); 
      
      setWirelessState(nextWifiState); 
      lastWifiBtn = now;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void app_main(void) {
  esp_task_wdt_config_t twdt_config = { 
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000, 
    .idle_core_mask = (1 << 0) | (1 << 1), 
    .trigger_panic = true 
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);

  gpio_config_t io_out = {
    .pin_bit_mask = (1ULL << PIN_TCI),
    .mode = GPIO_MODE_OUTPUT,
    .pull_down_en = 1,
    .pull_up_en = 0,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_out); 
  TCI_COIL_SPARK(); 
  
  gpio_config_t io_in = {
    .pin_bit_mask = (1ULL << PIN_BUTTON) | (1ULL << PIN_BTN_WIFI),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = 1,
    .pull_down_en = 0,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_in);

  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = LEDC_TIMER_10_BIT,
    .freq_hz          = 10,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .timer_sel      = LEDC_TIMER_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = PIN_TACH_OUT,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel);

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { 
    nvs_flash_erase(); 
    nvs_flash_init(); 
  }
  
  uart_config_t uart_cfg = { 
    .baud_rate = 250000, 
    .data_bits = UART_DATA_8_BITS, 
    .parity = UART_PARITY_DISABLE, 
    .stop_bits = UART_STOP_BITS_1, 
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE 
  };
  uart_param_config(UART_NUM_2, &uart_cfg); 
  uart_set_pin(UART_NUM_2, PIN_TX_TELEMETRY, PIN_RX_TELEMETRY, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, 512, 512, 0, NULL, 0);

  adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 }; 
  adc_oneshot_new_unit(&init_config1, &adc1_handle);
  
  adc_oneshot_chan_cfg_t config = { 
    .bitwidth = ADC_BITWIDTH_12, 
    .atten = ADC_ATTEN_TARGET 
  };
  adc_oneshot_config_channel(adc1_handle, ADC_CHAN_TPS, &config); 
  adc_oneshot_config_channel(adc1_handle, ADC_CHAN_BATT, &config); 
  adc_oneshot_config_channel(adc1_handle, ADC_CHAN_TEMP, &config);

  adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_TARGET,
    .bitwidth = ADC_BITWIDTH_12,
  };
  adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);

  gptimer_config_t timer_cfg = { 
    .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
    .direction = GPTIMER_COUNT_UP, 
    .resolution_hz = 1000000 
  };
  gptimer_new_timer(&timer_cfg, &timerDelay); 
  gptimer_new_timer(&timer_cfg, &timerDwell); 
  gptimer_new_timer(&timer_cfg, &timerSafetyDwell);
  
  gptimer_event_callbacks_t cbs_del = { .on_alarm = onDelayTimer }; 
  gptimer_register_event_callbacks(timerDelay, &cbs_del, NULL);
  gptimer_event_callbacks_t cbs_dwl = { .on_alarm = onDwellTimer }; 
  gptimer_register_event_callbacks(timerDwell, &cbs_dwl, NULL);
  gptimer_event_callbacks_t cbs_sft = { .on_alarm = onSafetyDwellTimer }; 
  gptimer_register_event_callbacks(timerSafetyDwell, &cbs_sft, NULL);
  
  gptimer_enable(timerDelay); 
  gptimer_enable(timerDwell); 
  gptimer_enable(timerSafetyDwell);

  esp_netif_init(); 
  esp_event_loop_create_default(); 
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); 
  esp_wifi_init(&cfg);
  
  wifi_config_t wifi_config = { 
    .ap = { 
      .ssid = AP_SSID, 
      .ssid_len = strlen(AP_SSID), 
      .password = AP_PASS, 
      .max_connection = 4, 
      .authmode = WIFI_AUTH_WPA_WPA2_PSK 
    } 
  };
  esp_wifi_set_mode(WIFI_MODE_AP); 
  esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
  
  loadCustomMap();

  xTaskCreatePinnedToCore(codeTaskNetwork, "TaskNet", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(codeTaskTelemetry, "TaskTel", 2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(codeTaskCommandListener, "TaskCmd", 2048, NULL, 2, NULL, 0);
  
  xTaskCreatePinnedToCore(codeTaskSensor, "TaskSens", 3072, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(codeTaskTachOutput, "TaskTach", 2048, NULL, 2, NULL, 1);
  
  gpio_config_t io_puls = { 
    .pin_bit_mask = (1ULL << PIN_PULSER), 
    .mode = GPIO_MODE_INPUT, 
    .pull_up_en = 1, 
    .pull_down_en = 0, 
    .intr_type = GPIO_INTR_NEGEDGE 
  };
  gpio_config(&io_puls); 

  gpio_install_isr_service(ESP_INTR_FLAG_IRAM); 
  gpio_isr_handler_add(PIN_PULSER, pulserISR, NULL);
}
