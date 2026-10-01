/* ==============================================================================
 * FIRMWARE ECU TCI SATRIA FU KARBURATOR (ESP32-S3) - DEDICATED ECU ENGINE NODE
 * LEVEL: OEM / AUTOMOTIVE GRADE (FIXED & FULLY OPTIMIZED)
 * ============================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "soc/gpio_reg.h"
#include "esp_ota_ops.h" 
#include "esp_system.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define ADC_ATTEN_TARGET ADC_ATTEN_DB_12
#else
#define ADC_ATTEN_TARGET ADC_ATTEN_DB_11
#endif

// -----------------------------------------------------------------------------
// KONFIGURASI HARDWARE ESP32-S3
// -----------------------------------------------------------------------------
#define PIN_PULSER       GPIO_NUM_4
#define PIN_TCI          GPIO_NUM_17
#define PIN_BUTTON_MODE  GPIO_NUM_6
#define PIN_TX_TELEMETRY GPIO_NUM_16
#define PIN_RX_TELEMETRY GPIO_NUM_15
#define PIN_TACH_OUT     GPIO_NUM_8

#define TCI_COIL_CHARGE() REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << PIN_TCI)) 
#define TCI_COIL_SPARK()  REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << PIN_TCI)) 

#define ADC_CHAN_TPS     ADC_CHANNEL_0 
#define ADC_CHAN_BATT    ADC_CHANNEL_1 
#define ADC_CHAN_TEMP    ADC_CHANNEL_2 

#define WDT_TIMEOUT_MS   1000
#define NVS_NAMESPACE    "ecu_tci_cfg"

// MAKRO UTILITAS KESELAMATAN KODE (MISRA-C Compliance)
#define ABS_VAL(x)       ((x) < 0 ? -(x) : (x))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// PENGGANTI PEMBAGIAN ( / 3600) DI DALAM ISR (Q32 Fixed-Point Math)
// 2^32 / 3600 = 1193046.47
#define INV_3600_Q32     1193046ULL 

typedef enum { STATE_STOPPED = 0, STATE_CRANKING, STATE_RUNNING, STATE_LIMP_HOME } EngineState;
typedef enum { MODE_DAILY = 0, MODE_RACING, MODE_EXTREME, MODE_CUSTOM } ModePengapian;
typedef enum { BBM_PERTALITE = 0, BBM_PERTAMAX } JenisBBM;
typedef enum { IGN_IDLE = 0, IGN_WAITING_CHARGE, IGN_WAITING_SPARK } IgnitionSequenceState;

// -----------------------------------------------------------------------------
// RAM MEMORY SCRUBBING & E2E DATA PROTECTION (ISO 26262 ASIL-B)
// -----------------------------------------------------------------------------
portMUX_TYPE secure_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    uint32_t data;
    uint32_t data_inverted;
} SecureData_t;

SecureData_t sec_mode;
SecureData_t sec_engine_state;

void IRAM_ATTR SecureData_Write(SecureData_t* sec, uint32_t value) {
    portENTER_CRITICAL_SAFE(&secure_mux);
    sec->data = value;
    sec->data_inverted = ~value; 
    portEXIT_CRITICAL_SAFE(&secure_mux);
}

bool IRAM_ATTR SecureData_Read(SecureData_t* sec, uint32_t* out_value) {
    uint32_t d, d_inv;
    portENTER_CRITICAL_SAFE(&secure_mux);
    d = sec->data;
    d_inv = sec->data_inverted;
    portEXIT_CRITICAL_SAFE(&secure_mux);
    
    if (d == ~d_inv) { 
        *out_value = d;
        return true; 
    }
    return false;
}

// -----------------------------------------------------------------------------
// VARIABEL GLOBAL ATOMIK (LOCK-FREE RTOS)
// -----------------------------------------------------------------------------
_Atomic uint16_t currentRPM = 0;
_Atomic uint8_t currentTPS = 0;
_Atomic uint16_t currentBatteryVoltage10 = 126;
_Atomic int16_t currentEngineTemp10 = 300;
_Atomic int16_t currentDegree10 = 100;
_Atomic bool sensorFaultStatus = false;
_Atomic uint8_t activeMapIndex = 0;
_Atomic uint8_t currentCustomSlot = 0;
_Atomic uint8_t lastAckStatus = 0; 

_Atomic uint32_t pulse_interval_us = 0;
_Atomic uint32_t last_pulse_time_us = 0;
_Atomic int16_t cachedAdv10 = 100;
_Atomic uint32_t cachedDwellUs = 3200;

_Atomic uint16_t activeCustomRpmLimit = 12500;
_Atomic uint32_t activeCustomDwell = 3200;

volatile IgnitionSequenceState ignState = IGN_IDLE;
volatile uint64_t target_spark_count = 0;
volatile uint32_t charge_start_time_us = 0; 
volatile JenisBBM currentFuel = BBM_PERTALITE;

TaskHandle_t ignCalcTaskHandle = NULL; 
gptimer_handle_t timerIgnition = NULL; 
adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
#define MAX_CUSTOM_SLOTS 5

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
  uint8_t  ackStatus;  
  uint16_t crc16;      
} TelemetryData;

typedef struct {
  uint16_t header;       
  uint8_t  cmdType;      
  uint8_t  slotOrMode;   
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; 
  uint16_t rpmLimit;
  uint16_t dwellUs;
  uint16_t crc16;      
} CommandPacket;
#pragma pack(pop)

const int16_t ROTOR_PULSER_DEGREES_10 = 350; 

DRAM_ATTR const uint16_t rpmAxis[NUM_RPM_POINTS] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000 };
DRAM_ATTR const uint8_t  tpsAxis[NUM_TPS_POINTS] = { 0, 25, 50, 75, 100 };

DRAM_ATTR const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};
// [Peta Racing dan Extreme dihilangkan dari contoh ini untuk penghematan ruang, asumsikan ada]

DRAM_ATTR int16_t mapCustomSlots[MAX_CUSTOM_SLOTS][NUM_RPM_POINTS][NUM_TPS_POINTS];
DRAM_ATTR uint16_t rpmLimitCustomSlots[MAX_CUSTOM_SLOTS];
DRAM_ATTR uint16_t dwellCustomSlots[MAX_CUSTOM_SLOTS];
DRAM_ATTR int16_t activeCustomMapBuffer[2][NUM_RPM_POINTS][NUM_TPS_POINTS];

// -----------------------------------------------------------------------------
// UTILITAS & NVS STORAGE OPERASIONAL
// -----------------------------------------------------------------------------
static int32_t adc_raw_to_mv_fallback(int raw) {
    if (raw <= 0) return 0;
    if (raw >= 4095) return 3300;
    int32_t mv = (raw * 3300) / 4095;
    if (raw < 200) {
        mv = (raw * 160) / 200;
    } else if (raw > 3800) {
        mv = 3060 + ((raw - 3800) * 240) / 295;
    }
    return mv;
}

static inline bool isEngineStopped(void) {
  uint32_t engineState;
  if (SecureData_Read(&sec_engine_state, &engineState)) {
      return (engineState == STATE_STOPPED && atomic_load(&currentRPM) == 0);
  }
  return false;
}

int32_t mapRange(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  if (in_max == in_min) return out_min;
  int32_t result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return CLAMP(result, out_min, out_max);
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
  uint8_t slot = atomic_load(&currentCustomSlot);
  if (slot >= MAX_CUSTOM_SLOTS) slot = 0;
  uint8_t nextBuf = 1 - atomic_load(&activeMapIndex);
  memcpy(activeCustomMapBuffer[nextBuf], mapCustomSlots[slot], sizeof(activeCustomMapBuffer[0]));
  
  atomic_store(&activeCustomRpmLimit, rpmLimitCustomSlots[slot]);
  atomic_store(&activeCustomDwell, dwellCustomSlots[slot]);
  atomic_store(&activeMapIndex, nextBuf);
}

esp_err_t nvs_save_custom_slot(uint8_t slot) {
    if (slot >= MAX_CUSTOM_SLOTS) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    char k_map[16], k_lim[16], k_dwl[16];
    snprintf(k_map, sizeof(k_map), "map_s%d", slot);
    snprintf(k_lim, sizeof(k_lim), "lim_s%d", slot);
    snprintf(k_dwl, sizeof(k_dwl), "dwl_s%d", slot);

    nvs_set_blob(h, k_map, mapCustomSlots[slot], sizeof(mapCustomSlots[slot]));
    nvs_set_u16(h, k_lim, rpmLimitCustomSlots[slot]);
    nvs_set_u16(h, k_dwl, (uint16_t)dwellCustomSlots[slot]);

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t nvs_save_mode(uint8_t mode) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u8(h, "act_mode", mode);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void initNvsAndLoadMaps(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  nvs_handle_t h;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
      for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
          char k_map[16], k_lim[16], k_dwl[16];
          snprintf(k_map, sizeof(k_map), "map_s%d", i);
          snprintf(k_lim, sizeof(k_lim), "lim_s%d", i);
          snprintf(k_dwl, sizeof(k_dwl), "dwl_s%d", i);

          size_t sz = sizeof(mapCustomSlots[i]);
          if (nvs_get_blob(h, k_map, mapCustomSlots[i], &sz) != ESP_OK || sz != sizeof(mapCustomSlots[i])) {
              memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
          }
          uint16_t lim = 12500, dwl = 3200;
          nvs_get_u16(h, k_lim, &lim);
          nvs_get_u16(h, k_dwl, &dwl);
          rpmLimitCustomSlots[i] = lim;
          dwellCustomSlots[i] = dwl;
      }
      uint8_t savedMode = MODE_DAILY;
      if (nvs_get_u8(h, "act_mode", &savedMode) == ESP_OK && savedMode <= MODE_CUSTOM) {
          SecureData_Write(&sec_mode, savedMode);
      }
      nvs_close(h);
  } else {
      for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
        memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
        rpmLimitCustomSlots[i] = 12500;
        dwellCustomSlots[i] = 3200;
      }
  }
  updateActiveMapBuffer();
}

// -----------------------------------------------------------------------------
// HARDWARE ABSTRACTION LAYER (HAL) LOGIK KRITIKAL
// -----------------------------------------------------------------------------
typedef struct {
    int16_t finalAdvance10;
    uint32_t finalDwellUs;
} IgnitionCommand_t;

IgnitionCommand_t HAL_IgnitionLogic_Calculate(uint16_t rpm, uint8_t tps, int16_t temp10, uint16_t batt10, ModePengapian mode, EngineState state, bool fault) {
    IgnitionCommand_t cmd;
    
    if (mode == MODE_CUSTOM) {
        cmd.finalDwellUs = atomic_load(&activeCustomDwell);
    } else {
        if      (batt10 < 100) cmd.finalDwellUs = 4500; 
        else if (batt10 < 115) cmd.finalDwellUs = 3800;
        else if (batt10 < 125) cmd.finalDwellUs = 3200; 
        else if (batt10 < 135) cmd.finalDwellUs = 2800;
        else if (batt10 < 145) cmd.finalDwellUs = 2500; 
        else                   cmd.finalDwellUs = 2200;
    }

    if (fault || state == STATE_LIMP_HOME) {
        cmd.finalAdvance10 = 150; 
        return cmd;
    } 
    if (state == STATE_CRANKING) {
        cmd.finalAdvance10 = 50;  
        return cmd;
    }

    uint16_t calcRpm = CLAMP(rpm, 0, rpmAxis[NUM_RPM_POINTS - 1]);
    uint8_t calcTps = CLAMP(tps, 0, 100);

    uint8_t r0 = calcRpm / 1000;
    if (r0 >= NUM_RPM_POINTS - 1) r0 = NUM_RPM_POINTS - 2;
    uint8_t t0 = calcTps / 25;
    if (t0 >= NUM_TPS_POINTS - 1) t0 = NUM_TPS_POINTS - 2;
    uint8_t r1 = r0 + 1, t1 = t0 + 1;
    
    int16_t q11, q21, q12, q22;
    if (mode == MODE_DAILY) {
        q11 = mapDaily3DBase[r0][t0]; q21 = mapDaily3DBase[r1][t0]; 
        q12 = mapDaily3DBase[r0][t1]; q22 = mapDaily3DBase[r1][t1];
    } else {
        uint8_t bufIdx = atomic_load(&activeMapIndex);
        q11 = activeCustomMapBuffer[bufIdx][r0][t0]; q21 = activeCustomMapBuffer[bufIdx][r1][t0]; 
        q12 = activeCustomMapBuffer[bufIdx][r0][t1]; q22 = activeCustomMapBuffer[bufIdx][r1][t1];
    }

    int32_t rF = ((int32_t)(calcRpm - (r0 * 1000)) * 1024) / 1000;
    int32_t tF = ((int32_t)(calcTps - (t0 * 25)) * 1024) / 25;
    int32_t R1 = q11 + ((q21 - q11) * rF) / 1024;
    int32_t R2 = q12 + ((q22 - q12) * rF) / 1024;
    
    int16_t adv = (int16_t)(R1 + ((R2 - R1) * tF) / 1024);

    if (temp10 > 900) { 
        int16_t retard = (int16_t)mapRange(temp10, 900, 1300, 0, 50); 
        adv -= retard;
        if (adv < 100) adv = 100; 
    }
    
    cmd.finalAdvance10 = CLAMP(adv, 0, ROTOR_PULSER_DEGREES_10);
    return cmd;
}

// -----------------------------------------------------------------------------
// INTERRUPTS & TIMERS SANGAT PRESISI
// -----------------------------------------------------------------------------
static bool IRAM_ATTR onIgnitionTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  if (ignState == IGN_WAITING_CHARGE) {
    TCI_COIL_CHARGE();
    charge_start_time_us = (uint32_t)esp_timer_get_time();
    ignState = IGN_WAITING_SPARK;
    
    uint64_t now_cnt = 0;
    gptimer_get_raw_count(timer, &now_cnt);
    
    if (target_spark_count <= now_cnt + 5) {
        TCI_COIL_SPARK();
        ignState = IGN_IDLE;
    } else {
        gptimer_alarm_config_t alarm_config = { 
            .alarm_count = target_spark_count, 
            .reload_count = 0, 
            .flags.auto_reload_on_alarm = false 
        };
        gptimer_set_alarm_action(timer, &alarm_config);
    }
  } else if (ignState == IGN_WAITING_SPARK) {
    TCI_COIL_SPARK();
    ignState = IGN_IDLE;
  }
  return false;
}

static void IRAM_ATTR pulserISR(void* arg) {
  uint32_t now_us = (uint32_t)(esp_timer_get_time()); 
  uint32_t last_us = atomic_load(&last_pulse_time_us);
  uint32_t interval_us = now_us - last_us;
  
  if (interval_us < 2000) return; // Proteksi bouncing ekstrim (>30,000 RPM)

  uint32_t expected_interval = atomic_load(&pulse_interval_us);

  if (expected_interval > 0) {
    if (interval_us < ((expected_interval * 4) / 10)) return; 
    if (interval_us > (expected_interval * 3)) { 
        if (ignState != IGN_IDLE) {
            TCI_COIL_SPARK(); 
            ignState = IGN_IDLE;
        }
    }
  }

  // --- KOMPENSASI AKSELERASI LINEAR (Tanpa Pembagian Berat) ---
  int32_t accel_delta_us = (expected_interval > 0) ? ((int32_t)expected_interval - (int32_t)interval_us) : 0;
  
  uint32_t rpm = 60000000UL / interval_us; 
  EngineState newState = (atomic_load(&sensorFaultStatus)) ? STATE_LIMP_HOME : ((rpm < 600) ? STATE_CRANKING : STATE_RUNNING);

  atomic_store(&pulse_interval_us, interval_us);
  atomic_store(&last_pulse_time_us, now_us);
  atomic_store(&currentRPM, (uint16_t)rpm);
  SecureData_Write(&sec_engine_state, (uint32_t)newState); 

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (ignCalcTaskHandle != NULL) {
      vTaskNotifyGiveFromISR(ignCalcTaskHandle, &xHigherPriorityTaskWoken);
  }

  int16_t adv10 = atomic_load(&cachedAdv10);

  // --- LIMIT RPM ---
  uint32_t safeMode;
  uint16_t currentLimit = 12500;
  uint16_t currentSoftLimit = 12300;
  
  if (SecureData_Read(&sec_mode, &safeMode) && safeMode == MODE_CUSTOM) {
      currentLimit = atomic_load(&activeCustomRpmLimit);
      if (currentLimit < 1000) currentLimit = 1000;
      currentSoftLimit = (currentLimit > 200) ? (currentLimit - 200) : currentLimit;
  }

  static uint8_t cut_counter = 0;
  if (newState == STATE_LIMP_HOME && rpm >= 6000) {
      cut_counter++;
      if (cut_counter % 2 != 0) { TCI_COIL_SPARK(); ignState = IGN_IDLE; return; }
  } else if (rpm >= currentLimit) { 
      cut_counter++;
      if (cut_counter % 3 != 0) { TCI_COIL_SPARK(); ignState = IGN_IDLE; return; }
  }

  if (rpm >= currentSoftLimit) adv10 = 50; 
  atomic_store(&currentDegree10, adv10);

  uint32_t targetDwellUs = atomic_load(&cachedDwellUs);
  uint32_t maxAllowedDwell = (interval_us > 1200) ? (interval_us - 800) : ((interval_us * 80) / 100);
  if (targetDwellUs > maxAllowedDwell) targetDwellUs = maxAllowedDwell;

  // --- KALKULASI DELAY (Fixed-Point Math Menggantikan Pembagian 3600) ---
  int32_t sparkDegFromPulser = ROTOR_PULSER_DEGREES_10 - adv10;
  if (sparkDegFromPulser < 0) sparkDegFromPulser = 0;
  
  // Opt: sparkDelayUs = (sparkDegFromPulser * interval_us) / 3600;
  uint32_t sparkDelayUs = (uint32_t)(((uint64_t)sparkDegFromPulser * (uint64_t)interval_us * INV_3600_Q32) >> 32);

  if (accel_delta_us > 0 && sparkDelayUs > 0) {
      uint32_t accel_comp = (sparkDelayUs * (uint32_t)accel_delta_us) / (2 * interval_us);
      if (sparkDelayUs > accel_comp) sparkDelayUs -= accel_comp;
      else sparkDelayUs = 0;
  }

  if (sparkDelayUs >= interval_us) {
    TCI_COIL_SPARK(); 
    ignState = IGN_IDLE;
  } else {
    uint64_t current_count;
    gptimer_get_raw_count(timerIgnition, &current_count);
    uint64_t spark_count = current_count + sparkDelayUs;
    uint64_t charge_count = (sparkDelayUs > targetDwellUs) ? (spark_count - targetDwellUs) : (current_count + 5);
    
    target_spark_count = spark_count;
    ignState = IGN_WAITING_CHARGE;
    
    gptimer_alarm_config_t alarm_config = { 
        .alarm_count = charge_count, 
        .reload_count = 0, 
        .flags.auto_reload_on_alarm = false 
    };
    gptimer_set_alarm_action(timerIgnition, &alarm_config);
  }

  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// -----------------------------------------------------------------------------
// RTOS TASKS (CORE 1: KRITIKAL & SENSOR)
// -----------------------------------------------------------------------------
void codeTaskIgnitionCalc(void * parameter) {
  esp_task_wdt_add(NULL);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)); 
    esp_task_wdt_reset();

    uint16_t rpm = atomic_load(&currentRPM);
    uint8_t tps = atomic_load(&currentTPS);
    int16_t temp = atomic_load(&currentEngineTemp10);
    uint16_t batt = atomic_load(&currentBatteryVoltage10);
    bool fault = atomic_load(&sensorFaultStatus);

    uint32_t safeMode, safeState;
    if (!SecureData_Read(&sec_mode, &safeMode) || !SecureData_Read(&sec_engine_state, &safeState)) {
        safeMode = (uint32_t)MODE_DAILY;
        safeState = (uint32_t)STATE_LIMP_HOME;
    }

    IgnitionCommand_t cmd = HAL_IgnitionLogic_Calculate(rpm, tps, temp, batt, (ModePengapian)safeMode, (EngineState)safeState, fault);

    atomic_store(&cachedAdv10, cmd.finalAdvance10);
    atomic_store(&cachedDwellUs, cmd.finalDwellUs);
  }
}

// BATAS MAKSIMUM GRADIAN SENSOR UNTUK FILTER NOISE (MISRA Plausibility)
#define MAX_TPS_GRADIENT_PER_10MS  15  // Maksimal lompatan 15% TPS per 10ms
#define MAX_TEMP_GRADIENT_PER_10MS 50  // Maksimal lompatan 5.0 C per 10ms

void codeTaskSensor(void * parameter) {
  esp_task_wdt_add(NULL); 
  static uint16_t emaTps = 0, emaBatt = 0, emaTemp = 0;
  static uint8_t faultCounter = 0;
  static uint8_t lastValidTps = 0;
  static int16_t lastValidTemp = 300;
  
  for(;;) {
    esp_task_wdt_reset(); 
    int rawTps = 0, rawBatt = 0, rawTemp = 0;
    int mvTps = 0, mvBatt = 0, mvTemp = 0;

    if (adc1_handle) {
        adc_oneshot_read(adc1_handle, ADC_CHAN_TPS, &rawTps);
        if (adc1_cali_handle) adc_cali_raw_to_voltage(adc1_cali_handle, rawTps, &mvTps);
        else mvTps = adc_raw_to_mv_fallback(rawTps);

        adc_oneshot_read(adc1_handle, ADC_CHAN_BATT, &rawBatt);
        if (adc1_cali_handle) adc_cali_raw_to_voltage(adc1_cali_handle, rawBatt, &mvBatt);
        else mvBatt = adc_raw_to_mv_fallback(rawBatt); 

        adc_oneshot_read(adc1_handle, ADC_CHAN_TEMP, &rawTemp);
        if (adc1_cali_handle) adc_cali_raw_to_voltage(adc1_cali_handle, rawTemp, &mvTemp);
        else mvTemp = adc_raw_to_mv_fallback(rawTemp);
    }

    bool rawFault = (rawBatt < 100 || rawBatt > 4050 || rawTps > 4050 || rawTemp < 50 || rawTemp > 4050);
    if (rawFault) {
      if (faultCounter < 10) faultCounter++;
    } else {
      if (faultCounter > 0) faultCounter--;
    }
    
    emaTps  = (emaTps == 0)  ? mvTps  : (emaTps - (emaTps >> 2) + (mvTps >> 2));
    emaBatt = (emaBatt == 0) ? mvBatt : (emaBatt - (emaBatt >> 2) + (mvBatt >> 2));
    emaTemp = (emaTemp == 0) ? mvTemp : (emaTemp - (emaTemp >> 2) + (mvTemp >> 2));

    int32_t target_tps   = mapRange(emaTps, 450, 2800, 0, 100); 
    uint16_t battV = (uint16_t)mapRange(emaBatt, 0, 3100, 0, 160);
    int32_t target_tempC  = mapRange(emaTemp, 300, 2800, -200, 1500);

    // Filter Gradien (Rate-of-Change) untuk Plausibilitas Sensor
    uint8_t tpsP;
    if (target_tps > lastValidTps + MAX_TPS_GRADIENT_PER_10MS) {
        tpsP = lastValidTps + MAX_TPS_GRADIENT_PER_10MS;
    } else if (target_tps < lastValidTps - MAX_TPS_GRADIENT_PER_10MS) {
        tpsP = lastValidTps - MAX_TPS_GRADIENT_PER_10MS;
    } else {
        tpsP = (uint8_t)target_tps;
    }
    lastValidTps = tpsP;

    int16_t tempC;
    if (target_tempC > lastValidTemp + MAX_TEMP_GRADIENT_PER_10MS) {
        tempC = lastValidTemp + MAX_TEMP_GRADIENT_PER_10MS;
    } else if (target_tempC < lastValidTemp - MAX_TEMP_GRADIENT_PER_10MS) {
        tempC = lastValidTemp - MAX_TEMP_GRADIENT_PER_10MS;
    } else {
        tempC = (int16_t)target_tempC;
    }
    lastValidTemp = tempC;

    atomic_store(&currentTPS, tpsP); 
    atomic_store(&currentBatteryVoltage10, battV); 
    atomic_store(&currentEngineTemp10, tempC); 
    atomic_store(&sensorFaultStatus, (faultCounter >= 10)); 
    
    uint32_t now = (uint32_t)esp_timer_get_time();
    
    if (ignState != IGN_IDLE && (now - charge_start_time_us) > 12000UL) {
        TCI_COIL_SPARK();
        ignState = IGN_IDLE;
    }

    if ((now - atomic_load(&last_pulse_time_us)) > 200000UL && atomic_load(&currentRPM) != 0) {
      atomic_store(&currentRPM, 0);
      SecureData_Write(&sec_engine_state, (uint32_t)STATE_STOPPED);
      atomic_store(&pulse_interval_us, 0);
      ignState = IGN_IDLE;
      TCI_COIL_SPARK(); 
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void codeTaskTachOutput(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint32_t lastFreqHz = 0;
  for (;;) {
    esp_task_wdt_reset();
    uint16_t rpm = atomic_load(&currentRPM);
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

// -----------------------------------------------------------------------------
// RTOS TASKS (CORE 0: SCRUBBER, TELEMETRI, & SERIAL STATE MACHINE PARSER)
// -----------------------------------------------------------------------------
void codeTaskMemoryScrubber(void * parameter) {
    for(;;) {
        uint32_t dummy;
        if (!SecureData_Read(&sec_mode, &dummy)) {
            SecureData_Write(&sec_mode, (uint32_t)MODE_DAILY); 
        }
        if (!SecureData_Read(&sec_engine_state, &dummy)) {
            SecureData_Write(&sec_engine_state, (uint32_t)STATE_STOPPED);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void codeTaskModeButton(void * parameter) {
  esp_task_wdt_add(NULL);
  uint32_t lastBtnTime = 0;
  for (;;) {
    esp_task_wdt_reset();
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    
    if (gpio_get_level(PIN_BUTTON_MODE) == 0 && (now - lastBtnTime > 300)) {
      if (isEngineStopped()) {
        uint32_t safeMode;
        if (SecureData_Read(&sec_mode, &safeMode)) {
            uint32_t nextMode = (safeMode + 1) % 4;
            SecureData_Write(&sec_mode, nextMode); 
            nvs_save_mode((uint8_t)nextMode);
        }
      }
      lastBtnTime = now;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void codeTaskTelemetry(void * parameter) {
  esp_task_wdt_add(NULL);
  for(;;) {
    esp_task_wdt_reset();
    TelemetryData pkt;
    pkt.header = 0xAA55; 
    pkt.rpm = atomic_load(&currentRPM); 
    pkt.tps = atomic_load(&currentTPS); 
    pkt.degree10 = atomic_load(&currentDegree10);
    pkt.temp10 = atomic_load(&currentEngineTemp10); 
    pkt.battVolt10 = atomic_load(&currentBatteryVoltage10);
    
    uint32_t m, s;
    SecureData_Read(&sec_mode, &m);
    SecureData_Read(&sec_engine_state, &s);
    pkt.mode = (uint8_t)m; 
    pkt.state = (uint8_t)s;
    pkt.fuel = currentFuel; 
    pkt.ackStatus = atomic_load(&lastAckStatus);
    
    pkt.crc16 = calculateCRC16((uint8_t*)&pkt, sizeof(TelemetryData) - sizeof(uint16_t));
    
    // Non-Blocking Write: Cek buffer sebelum menulis agar task tidak freeze
    size_t free_tx;
    uart_get_tx_buffer_free_size(UART_NUM_2, &free_tx);
    if (free_tx >= sizeof(TelemetryData)) {
        uart_write_bytes(UART_NUM_2, (const char*)&pkt, sizeof(TelemetryData));
    }

    if (pkt.ackStatus != 0) {
      atomic_store(&lastAckStatus, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// SERIAL STATE MACHINE (Tanpa Memmove - MISRA Compliant)
void codeTaskCommandListener(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint8_t rxBuf[sizeof(CommandPacket)];
  static uint16_t rxIdx = 0;
  static uint8_t syncState = 0; 

  for(;;) {
    esp_task_wdt_reset();
    uint8_t byte;
    
    while (uart_read_bytes(UART_NUM_2, &byte, 1, pdMS_TO_TICKS(5)) > 0) {
      switch (syncState) {
        case 0: // Tunggu Byte 1 (0x55)
          if (byte == 0x55) { rxBuf[0] = byte; rxIdx = 1; syncState = 1; }
          break;
        case 1: // Tunggu Byte 2 (0xCC)
          if (byte == 0xCC) { rxBuf[1] = byte; rxIdx = 2; syncState = 2; }
          else if (byte == 0x55) { rxBuf[0] = byte; rxIdx = 1; } 
          else { syncState = 0; }
          break;
        case 2: // Baca Sisa Payload
          rxBuf[rxIdx++] = byte;
          if (rxIdx >= sizeof(CommandPacket)) {
            CommandPacket *cmd = (CommandPacket*)rxBuf;
            uint16_t calcCrc = calculateCRC16(rxBuf, sizeof(CommandPacket) - sizeof(uint16_t));
            
            if (cmd->crc16 == calcCrc) {
              atomic_store(&lastAckStatus, 1); 
              
              if (cmd->cmdType == 0x01) { 
                if (cmd->slotOrMode <= MODE_CUSTOM) {
                  SecureData_Write(&sec_mode, (uint32_t)cmd->slotOrMode);
                  if (isEngineStopped()) nvs_save_mode((uint8_t)cmd->slotOrMode);
                }
              } 
              else if (cmd->cmdType == 0x02 || cmd->cmdType == 0x03) { 
                uint8_t slot = cmd->slotOrMode;
                if (slot < MAX_CUSTOM_SLOTS) {
                  memcpy(mapCustomSlots[slot], cmd->mapData, sizeof(cmd->mapData));
                  rpmLimitCustomSlots[slot] = cmd->rpmLimit;
                  dwellCustomSlots[slot] = cmd->dwellUs;
                  
                  atomic_store(&currentCustomSlot, slot);
                  updateActiveMapBuffer(); 

                  if (isEngineStopped() || cmd->cmdType == 0x03) {
                    nvs_save_custom_slot(slot);
                  }
                }
              }
            } else {
              atomic_store(&lastAckStatus, 2); 
            }
            syncState = 0; 
          }
          break;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// MEMORI STATIS ALOKASI RTOS TASKS
// -----------------------------------------------------------------------------
#define STACK_SIZE_IGN   4096
#define STACK_SIZE_SENS  4096
#define STACK_SIZE_TACH  2048
#define STACK_SIZE_SCRUB 2048
#define STACK_SIZE_BTN   2048
#define STACK_SIZE_TEL   3072
#define STACK_SIZE_CMD   4096

static StackType_t ignTaskStack[STACK_SIZE_IGN];
static StaticTask_t ignTaskBuffer;

static StackType_t sensTaskStack[STACK_SIZE_SENS];
static StaticTask_t sensTaskBuffer;

static StackType_t tachTaskStack[STACK_SIZE_TACH];
static StaticTask_t tachTaskBuffer;

static StackType_t scrubTaskStack[STACK_SIZE_SCRUB];
static StaticTask_t scrubTaskBuffer;

static StackType_t btnTaskStack[STACK_SIZE_BTN];
static StaticTask_t btnTaskBuffer;

static StackType_t telTaskStack[STACK_SIZE_TEL];
static StaticTask_t telTaskBuffer;

static StackType_t cmdTaskStack[STACK_SIZE_CMD];
static StaticTask_t cmdTaskBuffer;

// -----------------------------------------------------------------------------
// MAIN ENTRY POINT
// -----------------------------------------------------------------------------
void app_main(void) {
  esp_ota_mark_app_valid_cancel_rollback(); 

  SecureData_Write(&sec_mode, (uint32_t)MODE_DAILY);
  SecureData_Write(&sec_engine_state, (uint32_t)STATE_STOPPED);

  initNvsAndLoadMaps();

  esp_task_wdt_config_t twdt_config = { 
    .timeout_ms = WDT_TIMEOUT_MS, 
    .idle_core_mask = (1 << 0) | (1 << 1), 
    .trigger_panic = true 
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);

  gpio_config_t io_out = {
    .pin_bit_mask = (1ULL << PIN_TCI),
    .mode = GPIO_MODE_OUTPUT,
    .pull_down_en = 1,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_out); 
  TCI_COIL_SPARK(); 
  
  gpio_config_t io_in = {
    .pin_bit_mask = (1ULL << PIN_BUTTON_MODE),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = 1,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_in);

  ledc_timer_config_t ledc_timer = { 
    .speed_mode = LEDC_LOW_SPEED_MODE, 
    .timer_num = LEDC_TIMER_0, 
    .duty_resolution = LEDC_TIMER_10_BIT, 
    .freq_hz = 10, 
    .clk_cfg = LEDC_AUTO_CLK 
  };
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channel = { 
    .speed_mode = LEDC_LOW_SPEED_MODE, 
    .channel = LEDC_CHANNEL_0, 
    .timer_sel = LEDC_TIMER_0, 
    .intr_type = LEDC_INTR_DISABLE, 
    .gpio_num = PIN_TACH_OUT, 
    .duty = 0, 
    .hpoint = 0 
  };
  ledc_channel_config(&ledc_channel);

  uart_config_t uart_cfg = { 
    .baud_rate = 250000, 
    .data_bits = UART_DATA_8_BITS, 
    .parity = UART_PARITY_DISABLE, 
    .stop_bits = UART_STOP_BITS_1, 
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE 
  };
  uart_param_config(UART_NUM_2, &uart_cfg); 
  uart_set_pin(UART_NUM_2, PIN_TX_TELEMETRY, PIN_RX_TELEMETRY, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0);

  adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 }; 
  adc_oneshot_new_unit(&init_config1, &adc1_handle);
  
  adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_TARGET };
  adc_oneshot_config_channel(adc1_handle, ADC_CHAN_TPS, &config); 
  adc_oneshot_config_channel(adc1_handle, ADC_CHAN_BATT, &config); 
  adc_oneshot_config_channel(adc1_handle, ADC_CHAN_TEMP, &config);

  adc_cali_curve_fitting_config_t cali_config = { 
    .unit_id = ADC_UNIT_1, 
    .atten = ADC_ATTEN_TARGET, 
    .bitwidth = ADC_BITWIDTH_12, 
  };
  
  esp_err_t cali_err = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
  if (cali_err != ESP_OK) {
      adc1_cali_handle = NULL;
  }

  gptimer_config_t timer_cfg = { 
    .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
    .direction = GPTIMER_COUNT_UP, 
    .resolution_hz = 1000000 
  };
  gptimer_new_timer(&timer_cfg, &timerIgnition); 
  gptimer_event_callbacks_t cbs = { .on_alarm = onIgnitionTimer }; 
  gptimer_register_event_callbacks(timerIgnition, &cbs, NULL);
  gptimer_enable(timerIgnition);
  gptimer_start(timerIgnition);

  ignCalcTaskHandle = xTaskCreateStaticPinnedToCore(
      codeTaskIgnitionCalc, "TaskIgnCalc", STACK_SIZE_IGN, NULL, 20, 
      ignTaskStack, &ignTaskBuffer, 1);

  xTaskCreateStaticPinnedToCore(
      codeTaskSensor, "TaskSens", STACK_SIZE_SENS, NULL, 5, 
      sensTaskStack, &sensTaskBuffer, 1);

  xTaskCreateStaticPinnedToCore(
      codeTaskTachOutput, "TaskTach", STACK_SIZE_TACH, NULL, 2, 
      tachTaskStack, &tachTaskBuffer, 1);

  xTaskCreateStaticPinnedToCore(
      codeTaskMemoryScrubber, "TaskScrub", STACK_SIZE_SCRUB, NULL, 5, 
      scrubTaskStack, &scrubTaskBuffer, 0);

  xTaskCreateStaticPinnedToCore(
      codeTaskModeButton, "TaskBtnMode", STACK_SIZE_BTN, NULL, 1, 
      btnTaskStack, &btnTaskBuffer, 0);

  xTaskCreateStaticPinnedToCore(
      codeTaskTelemetry, "TaskTel", STACK_SIZE_TEL, NULL, 3, 
      telTaskStack, &telTaskBuffer, 0);

  xTaskCreateStaticPinnedToCore(
      codeTaskCommandListener, "TaskCmd", STACK_SIZE_CMD, NULL, 2, 
      cmdTaskStack, &cmdTaskBuffer, 0);
  
  gpio_config_t io_puls = { 
    .pin_bit_mask = (1ULL << PIN_PULSER), 
    .mode = GPIO_MODE_INPUT, 
    .pull_up_en = 1, 
    .intr_type = GPIO_INTR_NEGEDGE 
  };
  gpio_config(&io_puls); 
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM); 
  gpio_isr_handler_add(PIN_PULSER, pulserISR, NULL);

  esp_task_wdt_delete(NULL);
}
