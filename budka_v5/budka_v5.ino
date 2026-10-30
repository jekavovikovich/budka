/*
  ============================================================================
  ТЕРМОСТАТ БУДКИ НА ESP32-WROOM-32
  ============================================================================
  Версия с Wi-Fi и веб-сервером (встроенная WebServer.h, без сторонних
  библиотек). SSID/пароль и веб-страница - см. блок WI-FI / ВЕБ-СЕРВЕР ниже.
  mDNS не используется - "budka.local" резолвится через отдельную настройку
  на стороне роутера (не в прошивке).

  ---------------------------------------------------------------------------
  ВАЖНО: настройка TFT_eSPI выполняется НЕ в этом скетче, а в файле
  библиотеки User_Setup.h (обычно лежит в Arduino/libraries/TFT_eSPI/).
  Впишите туда (или создайте свой User_Setup, подключаемый через
  User_Setup_Select.h):

    #define ILI9341_DRIVER
    #define TFT_MISO 19
    #define TFT_MOSI 23
    #define TFT_SCLK 18
    #define TFT_CS   15
    #define TFT_DC    4
    #define TFT_RST  16
    #define TFT_BL   21
    #define TFT_BACKLIGHT_ON HIGH
    #define SPI_FREQUENCY  27000000

  Без этого экран работать не будет (или будет работать неправильно).
  ---------------------------------------------------------------------------

  ПРИНЦИП ПО FLOAT:
  DS18B20 через библиотеку DallasTemperature опрашивается через сырой
  формат sensors.getTemp() — это int16_t в единицах 1/128 °C, без float.
  Дальше всё пересчитывается в "десятые доли градуса" (int16_t) через
  целочисленную арифметику: tenths = raw * 5 / 64.
  Ни один float в рабочем цикле не используется.

  РАСПИНОВКА (см. предыдущее обсуждение):
    TFT (аппаратный VSPI): SCK18 MOSI23 CS15 DC4 RST16 LED21
    DS18B20 "коврик"  (аварийный)         GPIO13
    DS18B20 "воздух"  (опорный, термостат) GPIO14
    DS18B20 "улица"   (мониторинг)         GPIO27
    Энкодер: A=25 B=26 SW(кнопка)=33
    SSR  -> GPIO17
    EMR  -> GPIO32  (LOW  = катушка запитана = цепь ЗАМКНУТА, штатный режим;
                      HIGH = авария/сброс питания = цепь РАЗОМКНУТА, fail-safe;
                      на линии стоит подтяжка к 3,3 В, см. EMR_ENERGIZED_LEVEL)
    Датчик тока (ZMCT103C+LM358) -> GPIO34 (ADC1! НЕ ADC2, т.к. ADC2
                                              конфликтует с Wi-Fi)
  ============================================================================
*/

// ВАЖНО: WiFi/WebServer подключены ПЕРЕД TFT_eSPI намеренно - иначе
// возникает конфликт типа FS между библиотеками (TFT_eSPI тянет FS.h для
// поддержки SD-карт, и если это происходит раньше WebServer.h, тип FS
// оказывается виден только как fs::FS, а WebServer.h ожидает его в
// глобальном пространстве имён - ошибка компиляции "FS was not declared").
#include <WiFi.h>
#include <WebServer.h>
#include "time.h"
// WiFiManager (tzapu) - СТОРОННЯЯ библиотека, не входит в ядро ESP32.
// Установить через Arduino IDE: Sketch -> Include Library -> Manage Libraries -> "WiFiManager" (tzapu).
// Используется для captive-портала настройки Wi-Fi (см. блок НАСТРОЙКА WI-FI ниже).
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include "esp_ota_ops.h" // для esp_ota_mark_app_valid_cancel_rollback() - защита от битой прошивки, см. блок OTA
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Preferences.h>  // NVS - журнал перезагрузок (блок ДИАГНОСТИКА)
#include "esp_system.h"   // esp_reset_reason()
#include "esp_timer.h"    // esp_timer_get_time() - время работы без переполнения millis()
#include "esp_task_wdt.h"  // сторожевой таймер (watchdog) - см. блок WATCHDOG ниже
#include "soc/soc.h"       // REG_READ - прямое чтение регистра GPIO в ISR энкодера
#include "soc/gpio_reg.h"  // GPIO_IN_REG

// ---------------------------------------------------------------------------
// ПИНЫ
// ---------------------------------------------------------------------------
#define PIN_TFT_LED     21

#define PIN_DS_MAT      13   // датчик на коврике (аварийный, перегрев)
#define PIN_DS_AIR      14   // датчик воздуха на уровне собаки (опорный)
#define PIN_DS_OUT      27   // датчик "улица"

#define PIN_ENC_A       25
#define PIN_ENC_B       26
#define PIN_ENC_SW      33

#define PIN_SSR         17
#define PIN_EMR         32

#define PIN_CURRENT     34   // ADC1_CH6

// ---------------------------------------------------------------------------
// WI-FI / ВЕБ-СЕРВЕР
// ---------------------------------------------------------------------------
// Используются только встроенные в ядро ESP32 библиотеки (WiFi, WebServer) -
// никаких сторонних зависимостей ставить не нужно. mDNS не используется -
// резолвинг "budka.local" настраивается на стороне роутера отдельно.
//
// Учётные данные сети НЕ зашиваются в прошивку - устройство ориентируется
// только на то, что пользователь настроил через портал (см. блок НАСТРОЙКА
// WI-FI ЧЕРЕЗ ПОРТАЛ ниже). На "чистом" контроллере, где Wi-Fi ещё ни разу
// не настраивался, connectToWifi() просто не найдёт сохранённых данных, и
// подключение не состоится, пока пользователь вручную не запустит портал
// настройки (длинное нажатие энкодера) - устройство при этом продолжает
// полноценно работать через экран, веб-сервер просто не поднимется.
const char* WIFI_HOSTNAME = "budka"; // имя, под которым устройство представится роутеру по DHCP

const unsigned long WIFI_RETRY_INTERVAL_MS = 15000; // если не подключились - пробуем заново через это время

WebServer server(80);
bool webServerRunning = false; // сервер физически "поднят" (server.begin() вызван) только при реальном подключении к Wi-Fi

enum WifiConnState : uint8_t { WIFI_ST_CONNECTING, WIFI_ST_CONNECTED };
WifiConnState wifiState = WIFI_ST_CONNECTING;
unsigned long wifiStateChangeMs = 0;

// --- NTP / реальное время ---
// POSIX TZ-строка для Украины: EET (UTC+2) зимой, EEST (UTC+3) летом,
// переход в последнее воскресенье марта/октября - стандартное европейское
// правило, автоматический переход библиотека делает сама, вручную
// переключать не нужно. Если правило перехода в стране изменится
// законодательно - потребуется обновить только эту строку.
const char* TZ_UKRAINE = "EET-2EEST,M3.5.0/3,M10.5.0/4";
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.nist.gov";
const unsigned long TIME_RESYNC_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL; // раз в сутки

bool timeSynced = false;          // true - только если реальное время реально получено
bool timeSyncRequested = false;   // true - как только первая попытка была сделана (см. updateTimeSync())
unsigned long lastTimeSyncMs = 0;

// ---------------------------------------------------------------------------
// КОНСТАНТЫ ЛОГИКИ (подбираются/калибруются экспериментально)
// ---------------------------------------------------------------------------

// --- термостат ---
const int8_t  SETPOINT_MIN_C     = 5;
const int8_t  SETPOINT_MAX_C     = 30;
const int8_t  SETPOINT_DEFAULT_C = 16;
const int16_t HYSTERESIS_TENTHS  = 5;      // ±0.5°C

// --- аварийная температура коврика ---
const int16_t MAT_TEMP_ALARM_TENTHS = 480; // 48.0°C - выше этого разрыв ЭМР
                                            // (плёнка ограничена 40°C, запас)
// Перегрев подтверждается несколькими ПОДРЯД свежими чтениями (а не одним) -
// одиночный выброс из-за наводки не должен навсегда защёлкивать аварию.
// При опросе раз в TEMP_POLL_INTERVAL_MS (5 с) 2 чтения = ~10 с задержки.
const uint8_t MAT_OVERHEAT_CONFIRM_COUNT = 2;

// DS18B20 после сброса питания/сбоя отдаёт ровно 85.0 °C (значение по
// умолчанию регистра температуры). В будке такой температуры не бывает, так
// что это чтение всегда считается недействительным (см. isPlausibleRaw()).
const int16_t DS18B20_POWER_ON_RESET_RAW = 85 * 128; // 85.0 °C в единицах 1/128 °C

// Сколько ПОДРЯД хороших проверок нужно, чтобы автоматически снять "мягкую"
// аварию (отказ датчика воздуха/коврика, обрыв нагрузки). "Жёсткие" аварии
// (перегрев, превышение тока, залипание SSR) по-прежнему держатся до
// перезагрузки контроллера.
const uint8_t ALARM_RECOVER_GOOD_COUNT = 3;

// Если от опорного датчика воздуха не было ни одного успешного чтения дольше
// этого времени - его последнее значение считается устаревшим, и обычный
// гистерезис по нему больше не применяется (нагрев выключается), не дожидаясь
// полноценной аварии (она поднимается лишь после исчерпания переинициализаций).
const unsigned long AIR_DATA_STALE_MS = 90UL * 1000UL;

// --- калибровка RMS (в отсчётах АЦП) -> реальный ток в мА ---
// !!! ЗАГЛУШКИ. Откалибровать так: включить нагреватель, замерить реальный
// ток мультиметром в разрыв силовой цепи, одновременно посмотреть RMS.
// Serial-порт в продакшен-версии не инициализируется (см. setup()) - для
// калибровки временно добавьте Serial.begin(115200) в setup() и
// Serial.println(rms) в measureCurrentRms(), подставьте сюда пару значений
// (ток_в_мА, соответствующий_rms), затем уберите обратно !!!
const int32_t CURRENT_CALIB_MA_AT_RMS = 410; // ток в мА при калибровочном RMS ниже (TODO: замерить)
const int32_t CURRENT_CALIB_RMS_VALUE = 500; // RMS АЦП (в отсчётах), соответствующий указанному току (TODO: замерить)

// --- пороги аварий в реальных амперах (сотые доли, как и на экране: 100 = 1.00А) ---
// Считаются от уже откалиброванного значения тока (currentAmpsHundredths),
// а не от сырых единиц АЦП - так пороги напрямую соответствуют реальным амперам.
const int16_t CURRENT_OVERCURRENT_HUNDREDTHS = 100; // 1.00А - выше этого при включенном SSR = перегрузка/КЗ
const int16_t CURRENT_OPEN_LOAD_HUNDREDTHS   = 10;  // 0.10А - ниже этого при включенном SSR = обрыв нагрузки

// Порог "SSR залип" (ток есть, хотя SSR должен быть выключен) НЕ задаётся
// фиксированной константой - вместо этого при старте измеряется РЕАЛЬНЫЙ
// шумовой уровень датчика тока (пока SSR гарантированно выключен благодаря
// стартовой задержке), и порог считается как этот измеренный уровень ПЛЮС
// запас ниже. Так пороговое значение подстраивается под конкретный
// экземпляр платы датчика и не зависит от того, откалиброваны ли ещё
// CURRENT_CALIB_* константы - без этого фиксированный порог мог случайно
// оказаться ниже собственного шума платы и вызывать ложные срабатывания.
const int16_t CURRENT_LEAKAGE_MARGIN_HUNDREDTHS = 15; // запас сверх измеренного шума, 0.15А

const unsigned long CURRENT_CHECK_INTERVAL_MS = 1000; // как часто проверяем ток
const unsigned long CURRENT_SAMPLE_WINDOW_US  = 20000; // окно выборки ~20мс (1 период 50Гц)
const uint8_t CURRENT_FAULT_CONFIRM_COUNT = 3; // сколько подряд плохих проверок для срабатывания аварии

// как часто ОБНОВЛЯТЬ ЗНАЧЕНИЕ ТОКА НА ЭКРАНЕ (не путать с частотой
// внутренней проверки аварий CURRENT_CHECK_INTERVAL_MS выше - та чаще,
// это нужно для быстрой реакции на КЗ, а на экране обновляем реже)
const unsigned long CURRENT_DISPLAY_INTERVAL_MS = 5000;

// --- опрос температурных датчиков ---
const unsigned long TEMP_POLL_INTERVAL_MS = 5000;

// сколько подряд неудачных чтений датчика (ЛЮБОГО из трёх) считать поводом
// попробовать переинициализировать его (begin + повторный поиск адреса)
const uint16_t SENSOR_ERROR_REINIT_THRESHOLD = 20;

// максимум попыток переинициализации подряд, прежде чем для опорного
// датчика (воздух) поднимается авария ALARM_AIR_SENSOR_FAIL. Для остальных
// двух датчиков (коврик, улица) специальной аварии нет - после исчерпания
// попыток они просто продолжают показывать последнее значение с "?",
// как и раньше, но продолжают ЭТИ ЖЕ переинициализации при новых сериях ошибок.
const uint8_t SENSOR_REINIT_MAX_ATTEMPTS = 3;

// --- энкодер ---
const uint8_t ENCODER_STEPS_PER_CLICK = 4; // скорректировано по факту: 4 валидных перехода на 1 щелчок
const unsigned long LONG_PRESS_MS = 700;
const unsigned long DEBOUNCE_MS   = 30;

// --- EEPROM ---
const int EEPROM_SIZE = 24;
const int EEPROM_ADDR_MAGIC = 0;         // 1 байт - признак "данные валидны"
const int EEPROM_ADDR_SETPOINT = 1;      // 1 байт - уставка
const int EEPROM_ADDR_OTA_DECLINED_LEN = 3; // 1 байт - длина строки версии, отклонённой пользователем (0 = ничего не отклонено)
const int EEPROM_ADDR_OTA_DECLINED_STR = 4; // до 15 байт - сама строка версии (например "1.2.3"), без null-терминатора
const uint8_t EEPROM_MAGIC_VALUE = 0xA5;

// --- задержка включения ЭМР после старта ---
// ЭМР держится РАЗОМКНУТЫМ первые несколько секунд после включения
// контроллера, пока не завершится инициализация датчиков температуры -
// это исключает подачу питания на нагреватель до того, как система
// реально готова его контролировать.
const unsigned long EMR_STARTUP_DELAY_MS = 15000;

// --- задержка первого включения SSR после старта ---
// SSR не включается первые SSR_STARTUP_DELAY_MS после старта контроллера,
// ДАЖЕ ЕСЛИ по показаниям опорного датчика нужен нагрев. Это ДОПОЛНИТЕЛЬНАЯ
// задержка поверх задержки ЭМР (30с общая = 15с ЭМР + ещё 15с сверху) -
// то есть цепь физически замкнута заранее, но сама коммутация нагрева
// откладывается ещё немного для полной уверенности в готовности системы.
const unsigned long SSR_STARTUP_DELAY_MS = 30000;

// --- "слепое окно" для датчика тока после переключения SSR ---
// В момент включения/выключения SSR возможны кратковременные переходные
// процессы на линии тока (бросок при включении, затухание при выключении) -
// в течение этого времени после ЛЮБОГО переключения SSR проверка аварий
// по датчику тока приостанавливается, чтобы не ловить ложные срабатывания.
const unsigned long CURRENT_CHECK_BLANKING_MS = 5000;

// ---------------------------------------------------------------------------
// WATCHDOG (сторожевой таймер)
// ---------------------------------------------------------------------------
// Если loop() зависнет (например, застрянет сетевой вызов) при включённом
// SSR, нагрев остался бы включённым без какого-либо контроля тока и
// температуры. Сторожевой таймер перезагружает контроллер, если loop() не
// "отметился" за WDT_TIMEOUT_S секунд; после перезагрузки срабатывают
// стартовые задержки ЭМР/SSR (см. EMR_STARTUP_DELAY_MS, SSR_STARTUP_DELAY_MS).
//
// На время заведомо долгих блокирующих сетевых операций (проверка версии и
// скачивание прошивки по OTA) таймер временно отключается - см. WdtPause.
const uint32_t WDT_TIMEOUT_S = 10;
bool wdtEnabled = false;

void wdtStart() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  esp_task_wdt_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;   // следим только за задачей loop(), не за idle-задачами
  cfg.trigger_panic = true; // по срабатыванию - перезагрузка
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) {
    esp_task_wdt_init(&cfg);
  }
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL); // подписываем текущую задачу (loop)
  wdtEnabled = true;
}

inline void wdtFeed() {
  if (wdtEnabled) esp_task_wdt_reset();
}

// RAII-пауза: на время жизни объекта loop() не контролируется watchdog'ом,
// при выходе из области видимости (в т.ч. по любому return) контроль
// возвращается, а таймер "обнуляется".
struct WdtPause {
  bool wasEnabled;
  WdtPause() : wasEnabled(wdtEnabled) {
    if (wasEnabled) esp_task_wdt_delete(NULL);
  }
  ~WdtPause() {
    if (wasEnabled) {
      esp_task_wdt_add(NULL);
      esp_task_wdt_reset();
    }
  }
};

// ---------------------------------------------------------------------------
// ОБЪЕКТЫ
// ---------------------------------------------------------------------------
OneWire oneWireMat(PIN_DS_MAT);
OneWire oneWireAir(PIN_DS_AIR);
OneWire oneWireOut(PIN_DS_OUT);

DallasTemperature dsMat(&oneWireMat);
DallasTemperature dsAir(&oneWireAir);
DallasTemperature dsOut(&oneWireOut);

TFT_eSPI tft = TFT_eSPI();

// ---------------------------------------------------------------------------
// ГЛОБАЛЬНОЕ СОСТОЯНИЕ (позже пригодится для REST API)
// ---------------------------------------------------------------------------

// температуры в десятых долях градуса (например 235 = 23.5°C)
// значение сохраняет ПОСЛЕДНЕЕ УСПЕШНОЕ чтение - при ошибке чтения число
// не затирается, вместо этого выставляется флаг ошибки (см. ниже),
// -1270 остаётся только как "данных не было вообще ни разу"
int16_t tempMat  = -1270;
int16_t tempAir  = -1270;
int16_t tempOut  = -1270;

// флаги "последнее чтение не удалось" - используются, чтобы рисовать
// жёлтый "?" рядом со старым значением вместо стирания на "--.-"
bool tempMatError = true; // true до первого успешного чтения
bool tempAirError = true;
bool tempOutError = true;

// счётчик подряд идущих неудачных чтений опорного датчика (воздух) -
// используется для перевода временного сбоя связи в полноценную аварию
uint16_t airErrorStreak = 0;

// то же самое для коврика и улицы (у них нет отдельной аварии, но
// переинициализация датчика работает одинаково для всех трёх)
uint16_t matErrorStreak = 0;
uint16_t outErrorStreak = 0;

// счётчики ПОДРЯД идущих хороших чтений - для автосброса "мягких" аварий
uint8_t airGoodStreak = 0;
uint8_t matGoodStreak = 0;
uint8_t matOverheatCounter = 0;    // сколько подряд свежих чтений коврика выше порога
uint8_t openLoadGoodCounter = 0;   // сколько подряд проверок тока при включенном SSR в норме
unsigned long tempAirLastGoodMs = 0; // когда в последний раз успешно прочитан датчик воздуха
bool tempPollCompletedOnce = false;  // true после первого полностью завершённого цикла опроса

// сколько попыток переинициализации уже сделано подряд для каждого датчика
// (сбрасывается в 0 при любом успешном чтении)
uint8_t matReinitAttempts = 0;
uint8_t airReinitAttempts = 0;
uint8_t outReinitAttempts = 0;

// ток через нагреватель для ОТОБРАЖЕНИЯ (в сотых долях ампера, например
// 41 = 0.41А). Обновляется внутри checkCurrent() при каждой её проверке,
// но на экран "публикуется" (снимок для показа) реже - см. CURRENT_DISPLAY_INTERVAL_MS
int16_t currentAmpsHundredths        = 0;
int16_t currentAmpsForDisplay        = 0;
int16_t currentAmpsLastShown         = 32000; // сентинел, чтобы первая отрисовка точно произошла

int16_t tempMatLastShown = 32000; // заведомо невозможное значение, чтобы первая отрисовка точно произошла
int16_t tempAirLastShown = 32000;
int16_t tempOutLastShown = 32000;

// последнее ОТОБРАЖЁННОЕ состояние флага ошибки - отдельно от значения,
// т.к. ошибка может появиться/исчезнуть без изменения самого числа
bool tempMatErrorLastShown = false;
bool tempAirErrorLastShown = false;
bool tempOutErrorLastShown = false;

// то же самое для полей уставки/статуса нагрева/аварий - чтобы не мигали,
// если реального изменения значения не произошло
int8_t  lastShownSetpointValue  = -128; // сентинел, вне диапазона 5..30
bool    lastShownEditingMode    = false;
bool    lastShownSetpointValid  = false; // false = ещё ни разу не рисовали это поле

int8_t  lastShownSsrState       = -1;    // -1 = ещё не рисовали (валидные значения 0/1)
bool    lastShownEmergencyMode  = false; // для перерисовки индикатора "EM!" на строке статуса

uint16_t lastShownAlarmFlags    = 0xFFFF; // сентинел, вне диапазона реальных флагов (0..15)

int8_t setpointC = SETPOINT_DEFAULT_C;   // текущая активная уставка
int8_t pendingSetpointC = SETPOINT_DEFAULT_C; // уставка, которую крутят энкодером, но ещё не подтвердили

// если после начала редактирования уставки энкодером кнопка подтверждения
// не нажата в течение этого времени - редактирование отменяется, и
// значение возвращается к последней подтверждённой уставке
const unsigned long ENCODER_EDIT_TIMEOUT_MS = 10000;
unsigned long editingActivityMs = 0; // обновляется при входе в режим редактирования и на каждом клике
bool editingSetpoint = false;

bool ssrState = false;   // текущая команда SSR (true = включен нагрев)

// коды аварий (битовая маска, чтобы можно было хранить несколько одновременно)
enum AlarmBits : uint8_t {
  ALARM_NONE            = 0,
  ALARM_MAT_OVERHEAT    = 1 << 0,
  ALARM_OVERCURRENT     = 1 << 1,
  ALARM_SSR_STUCK       = 1 << 2,
  ALARM_OPEN_LOAD       = 1 << 3,
  ALARM_AIR_SENSOR_FAIL = 1 << 4,
  ALARM_MAT_SENSOR_FAIL = 1 << 5, // отказ датчика коврика после исчерпания переинициализаций -
                                  // НЕ останавливает нагрев сам по себе (см. updateThermostat()),
                                  // только предупреждение - опорный датчик воздуха продолжает
                                  // управлять термостатом как обычно
};
uint8_t alarmFlags = ALARM_NONE;
bool emrClosed = false; // текущее состояние ЭМР (true = цепь замкнута, штатно)
                        // изначально false - см. EMR_STARTUP_DELAY_MS ниже

unsigned long systemStartMs = 0; // фиксируется в setup()
bool systemReady = false;        // становится true через EMR_STARTUP_DELAY_MS после старта
bool ssrStartupDelayPassed = false; // становится true через SSR_STARTUP_DELAY_MS после старта

// --- аварийный таймерный режим обогрева ---
// Если опорный датчик воздуха отказал (ALARM_AIR_SENSOR_FAIL, после
// исчерпания переинициализаций) - обычный термостат управлять не может.
// Полностью останавливать нагрев в этом случае опасно само по себе, если
// на улице мороз - собака рискует замёрзнуть, пока никто не заметил
// неисправность. Поэтому при определённых условиях (см. emergencyModeEligible())
// нагрев продолжается по грубому таймеру вместо обратной связи по температуре.
const unsigned long EMERGENCY_ON_MS  = 15UL * 60UL * 1000UL; // 15 минут нагрев
const unsigned long EMERGENCY_OFF_MS = 15UL * 60UL * 1000UL; // 15 минут пауза

bool emergencyModeActive = false;       // true, пока реально работает аварийный таймер
bool emergencyTimerHeating = true;      // текущая фаза таймера (нагрев/пауза)
unsigned long emergencyPhaseStartMs = 0;
bool emergencyTimerInitialized = false; // чтобы фаза стартовала заново при каждом новом входе в режим

// Условия, при которых допустимо перейти на аварийный таймерный обогрев
// вместо полной остановки нагрева. Проверка тока (ALARM_OVERCURRENT/
// ALARM_SSR_STUCK) здесь не дублируется - структура updateThermostat()
// гарантирует, что эта функция вызывается только когда таких аварий нет.
bool emergencyModeEligible() {
  return (tempOut != -1270) && (tempOut < 0); // валидные данные с улицы ЕСТЬ и там мороз
}

// Продвигает грубый таймер 15/15 и возвращает, должен ли SSR сейчас греть.
bool updateEmergencyTimer() {
  unsigned long now = millis();
  if (!emergencyTimerInitialized) {
    emergencyPhaseStartMs = now;
    emergencyTimerHeating = true; // при входе в режим сразу начинаем с фазы нагрева
    emergencyTimerInitialized = true;
  }

  unsigned long phaseDuration = emergencyTimerHeating ? EMERGENCY_ON_MS : EMERGENCY_OFF_MS;
  if (now - emergencyPhaseStartMs >= phaseDuration) {
    emergencyTimerHeating = !emergencyTimerHeating;
    emergencyPhaseStartMs = now;
  }
  return emergencyTimerHeating;
}

unsigned long lastSsrTransitionMs = 0; // момент последнего переключения SSR - для "слепого окна" датчика тока

// счётчики подряд идущих "плохих" проверок тока (объявлены здесь, а не рядом
// с checkCurrent(), т.к. updateThermostat() тоже их сбрасывает при
// переключении SSR и находится выше по файлу)
uint8_t overcurrentCounter = 0;
uint8_t ssrStuckCounter    = 0;
uint8_t openLoadCounter    = 0;

bool backlightOn = true;

// --- режимы работы устройства (обычная работа / настройка Wi-Fi) ---
// Длинное нажатие при включённой подсветке в обычном режиме больше НЕ
// переключает подсветку (это теперь делает только автотаймер бездействия,
// см. BACKLIGHT_AUTO_OFF_MS ниже) - вместо этого запускает экран
// подтверждения входа в настройку Wi-Fi. Вся логика термостата/датчиков/
// безопасности работает ОДИНАКОВО во всех режимах - режим влияет только
// на то, что показывается на экране и как трактуются энкодер/кнопка.
enum SystemMode : uint8_t {
  MODE_NORMAL,             // обычная работа
  MODE_WIFI_SETUP_CONFIRM, // экран "Войти в настройку Wi-Fi? ДА/НЕТ"
  MODE_WIFI_SETUP_ACTIVE   // портал WiFiManager активен (AP + captive portal)
};
SystemMode systemMode = MODE_NORMAL;

bool wifiSetupConfirmYes = false; // по умолчанию выделено "НЕТ" - защита от случайного входа
unsigned long wifiSetupConfirmActivityMs = 0;
const unsigned long WIFI_SETUP_CONFIRM_TIMEOUT_MS = 10000; // 10с на решение - отмена по таймауту, как и для уставки

// --- автоматическое выключение подсветки при бездействии энкодера ---
const unsigned long BACKLIGHT_AUTO_OFF_MS = 10UL * 60UL * 1000UL; // 10 минут
unsigned long lastEncoderActivityMs = 0; // обновляется при вращении и любом нажатии кнопки

// ---------------------------------------------------------------------------
// ЛОГИЧЕСКИЕ УРОВНИ АКТИВАЦИИ SSR / EMR
// ---------------------------------------------------------------------------
// ВАЖНО: разные платы по-разному трактуют "включено". Голый SSR-чип обычно
// активен по HIGH, но в данном случае ЭКСПЕРИМЕНТАЛЬНО ПОДТВЕРЖДЕНО, что
// ОБА используемых модуля (и SSR, и EMR) срабатывают по LOW - то есть
// подача LOW на IN включает нагрузку/реле, HIGH - выключает.
//
// Если платы заменят на другие с иной логикой - достаточно поменять
// значения здесь, весь остальной код трогать не нужно.
const int SSR_ACTIVE_LEVEL      = LOW;  // уровень, при котором SSR ПРОПУСКАЕТ ток
const int SSR_INACTIVE_LEVEL    = HIGH;

const int EMR_ENERGIZED_LEVEL   = LOW;  // уровень, при котором катушка ЭМР ЗАПИТАНА (замкнуто)
const int EMR_DEENERGIZED_LEVEL = HIGH; // уровень аварии/сброса питания (разомкнуто)

// ---------------------------------------------------------------------------
// ЭНКОДЕР (устойчивый табличный квадратурный декодер)
// ---------------------------------------------------------------------------
// Наивное сравнение "A==B => +1 иначе -1" плохо переносит дребезг контактов
// и может пропускать/задваивать шаги. Вместо этого используем таблицу
// допустимых переходов состояний (стандартный приём для квадратурных
// энкодеров) - недопустимые/дребезговые переходы просто игнорируются.
//
// state = (A<<1)|B, индекс таблицы = (предыдущее_state<<2)|текущее_state
// Таблица инвертирована относительно "стандартной" (знаки поменяны местами),
// т.к. на практике для конкретного энкодера направление оказалось обратным.
//
// ВАЖНО: таблица помечена DRAM_ATTR - константные массивы по умолчанию лежат
// во флеш-памяти, а обработчик прерывания может сработать в момент записи во
// флеш (EEPROM.commit(), OTA), когда флеш недоступен - это вызвало бы сбой.
// По той же причине пины читаются прямым чтением регистра GPIO (оба сразу, за
// одну операцию), а не через digitalRead(), который не гарантированно
// размещён в IRAM.
DRAM_ATTR const int8_t ENCODER_TRANSITION_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

volatile uint8_t encoderPrevState = 0;
volatile int32_t encoderRawDelta = 0;
// Диагностика помех на линиях энкодера (см. блок ДИАГНОСТИКА): сколько раз
// сработало прерывание и сколько из них - "невозможные" переходы (изменились
// сразу оба сигнала или пропущен промежуточный шаг). При чистых линиях и
// крутящейся руке невозможных переходов почти нет; всплеск без вращения -
// признак наводок. Только счётчики, на работу энкодера не влияют.
volatile uint32_t encoderIsrCount = 0;
volatile uint32_t encoderInvalidCount = 0;

void IRAM_ATTR encoderISR() {
  uint32_t gpioIn = REG_READ(GPIO_IN_REG); // GPIO 0..31 одним чтением (пины 25 и 26 - в этом диапазоне)
  uint8_t a = (gpioIn >> PIN_ENC_A) & 1;
  uint8_t b = (gpioIn >> PIN_ENC_B) & 1;
  uint8_t currState = (a << 1) | b;
  uint8_t index = (encoderPrevState << 2) | currState;
  int8_t step = ENCODER_TRANSITION_TABLE[index];
  encoderIsrCount++;
  if (step == 0 && currState != encoderPrevState) encoderInvalidCount++;
  encoderRawDelta += step;
  encoderPrevState = currState;
}

// ---------------------------------------------------------------------------
// КНОПКА ЭНКОДЕРА (короткое/длинное нажатие, без delay())
// ---------------------------------------------------------------------------
bool buttonPrevRaw = HIGH;      // предполагаем подтяжку к HIGH, кнопка замыкает на GND
bool buttonStableState = HIGH;
unsigned long buttonLastChangeMs = 0;
unsigned long buttonPressStartMs = 0;
bool buttonIsDown = false;
bool longPressFired = false; // чтобы длинное нажатие срабатывало ОДИН раз, сразу по достижении порога

void handleButton() {
  bool raw = digitalRead(PIN_ENC_SW);

  if (raw != buttonPrevRaw) {
    buttonLastChangeMs = millis();
    buttonPrevRaw = raw;
  }

  if ((millis() - buttonLastChangeMs) > DEBOUNCE_MS && raw != buttonStableState) {
    buttonStableState = raw;

    if (buttonStableState == LOW) {
      // кнопка нажата
      buttonIsDown = true;
      buttonPressStartMs = millis();
      longPressFired = false;
    } else {
      // кнопка отпущена
      if (buttonIsDown) {
        unsigned long pressDuration = millis() - buttonPressStartMs;
        buttonIsDown = false;

        // короткое нажатие обрабатываем здесь, ТОЛЬКО если длинное ещё не
        // сработало (иначе после срабатывания длинного нажатия на отпускании
        // не должно дополнительно вызываться короткое)
        if (!longPressFired && pressDuration < LONG_PRESS_MS) {
          onShortPress();
        }
      }
    }
  }

  // проверка длинного нажатия ПОКА кнопка ещё удерживается - срабатывает
  // сразу по достижении порога, не дожидаясь отпускания кнопки
  if (buttonIsDown && !longPressFired && (millis() - buttonPressStartMs) >= LONG_PRESS_MS) {
    longPressFired = true;
    onLongPress();
  }
}

void onShortPress() {
  lastEncoderActivityMs = millis(); // любое нажатие считается активностью, независимо от результата

  if (!backlightOn) {
    // при выключенной подсветке ЛЮБОЕ нажатие (короткое или длинное) просто
    // будит экран и на этом всё - никакого другого действия оно не совершает
    backlightOn = true;
    digitalWrite(PIN_TFT_LED, HIGH);
    return;
  }

  switch (systemMode) {
    case MODE_NORMAL:
      if (editingSetpoint) {
        // подтверждение новой уставки
        setpointC = pendingSetpointC;
        editingSetpoint = false;
        saveSetpointToEeprom(setpointC);
      }
      // если не в режиме редактирования - короткое нажатие пока не используется
      break;

    case MODE_WIFI_SETUP_CONFIRM:
      // подтверждение текущего выбора (ДА/НЕТ)
      if (wifiSetupConfirmYes) {
        systemMode = MODE_WIFI_SETUP_ACTIVE;
        startWifiSetupPortal();
      } else {
        systemMode = MODE_NORMAL;
      }
      break;

    case MODE_WIFI_SETUP_ACTIVE:
      // короткое нажатие в активном портале пока не используется - выход
      // только длинным нажатием (см. onLongPress())
      break;
  }
}

void onLongPress() {
  lastEncoderActivityMs = millis(); // в т.ч. пробуждение экрана - это активность

  if (!backlightOn) {
    // при выключенной подсветке ЛЮБОЕ нажатие просто будит экран - вход в
    // настройку Wi-Fi вслепую не имеет смысла, поэтому длинное нажатие
    // здесь НЕ запускает экран подтверждения, только включает подсветку
    backlightOn = true;
    digitalWrite(PIN_TFT_LED, HIGH);
    return;
  }

  switch (systemMode) {
    case MODE_NORMAL:
      // подсветка теперь выключается ТОЛЬКО автотаймером бездействия
      // (см. checkBacklightAutoOff()) - длинное нажатие при включённой
      // подсветке запускает экран подтверждения входа в настройку Wi-Fi
      systemMode = MODE_WIFI_SETUP_CONFIRM;
      wifiSetupConfirmYes = false; // по умолчанию "НЕТ" - осторожная защита от случайного входа
      wifiSetupConfirmActivityMs = millis();
      break;

    case MODE_WIFI_SETUP_CONFIRM:
      // длинное нажатие здесь тоже трактуем как отмену - выходим сразу,
      // не дожидаясь подтверждения через короткое нажатие на "НЕТ"
      systemMode = MODE_NORMAL;
      break;

    case MODE_WIFI_SETUP_ACTIVE:
      // ручная отмена активного портала настройки
      exitWifiSetupMode();
      break;
  }
}

// ---------------------------------------------------------------------------
// EEPROM
// ---------------------------------------------------------------------------
void loadSetpointFromEeprom() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (magic == EEPROM_MAGIC_VALUE) {
    int8_t stored = (int8_t)EEPROM.read(EEPROM_ADDR_SETPOINT);
    if (stored >= SETPOINT_MIN_C && stored <= SETPOINT_MAX_C) {
      setpointC = stored;
      pendingSetpointC = stored;
    }
  }
}

void saveSetpointToEeprom(int8_t value) {
  // флеш-память изнашивается от записей - если там уже лежит то же значение,
  // ничего не пишем
  if (EEPROM.read(EEPROM_ADDR_MAGIC) == EEPROM_MAGIC_VALUE &&
      (int8_t)EEPROM.read(EEPROM_ADDR_SETPOINT) == value) {
    return;
  }
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);
  EEPROM.write(EEPROM_ADDR_SETPOINT, (uint8_t)value);
  EEPROM.commit();
}

// Версия прошивки, которую пользователь явно ОТКЛОНИЛ через веб-страницу -
// переживает перезагрузку, чтобы баннер "доступно обновление" не показывался
// снова и снова для уже отклонённой версии. При выходе следующей, более
// новой версии сравнение просто не совпадёт, и баннер закономерно появится
// заново - отдельно сбрасывать это значение не нужно.
String loadOtaDeclinedVersion() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (magic != EEPROM_MAGIC_VALUE) return "";

  uint8_t len = EEPROM.read(EEPROM_ADDR_OTA_DECLINED_LEN);
  if (len == 0 || len > 15) return "";

  char buf[16];
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = (char)EEPROM.read(EEPROM_ADDR_OTA_DECLINED_STR + i);
  }
  buf[len] = '\0';
  return String(buf);
}

void saveOtaDeclinedVersion(const String &version) {
  // то же самое, что и для уставки: не пишем во флеш, если версия уже сохранена
  if (loadOtaDeclinedVersion() == version) {
    return;
  }
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);

  uint8_t len = (uint8_t)min((size_t)15, version.length());
  EEPROM.write(EEPROM_ADDR_OTA_DECLINED_LEN, len);
  for (uint8_t i = 0; i < len; i++) {
    EEPROM.write(EEPROM_ADDR_OTA_DECLINED_STR + i, (uint8_t)version[i]);
  }
  EEPROM.commit();
}

// ---------------------------------------------------------------------------
// ДАТЧИКИ ТЕМПЕРАТУРЫ (без float)
// ---------------------------------------------------------------------------

// Переводит сырое значение DallasTemperature (1/128 °C) в десятые доли градуса.
// DEVICE_DISCONNECTED_RAW обрабатывается отдельно (сенсор не отвечает).
int16_t rawToTenths(int16_t raw) {
  if (raw == DEVICE_DISCONNECTED_RAW) {
    return -1270; // условный код "нет связи с датчиком" (-127.0°C, как у DallasTemperature)
  }
  // raw в единицах 1/128, переводим в 1/10 через целочисленную арифметику
  // с ОКРУГЛЕНИЕМ до ближайшего (а не усечением к нулю): 5/64 == 10/128
  int32_t t = (int32_t)raw * 5;
  t = (t >= 0) ? (t + 32) / 64 : (t - 32) / 64;
  return (int16_t)t;
}

// Чтение пригодно к использованию: датчик отвечает И это не "заводское"
// значение 85.0 °C, которое DS18B20 отдаёт после сброса питания (см.
// DS18B20_POWER_ON_RESET_RAW).
bool isPlausibleRaw(int16_t raw) {
  return raw != DEVICE_DISCONNECTED_RAW && raw != DS18B20_POWER_ON_RESET_RAW;
}

// ---------------------------------------------------------------------------
// Т.к. DallasTemperature::getTemp() требует явный адрес устройства,
// а у нас по одному датчику на шину - один раз при старте считываем адрес
// каждого датчика и сохраняем.
// ---------------------------------------------------------------------------
DeviceAddress addrMat, addrAir, addrOut;
bool matFound = false, airFound = false, outFound = false;

// интервал между стартом инициализации/конверсии разных датчиков -
// разносим по времени, чтобы не дёргать все три шины одновременно
const unsigned long SENSOR_STAGGER_MS = 300;

// время конверсии DS18B20 при 12-битном разрешении (по даташиту)
const unsigned long DS18B20_CONVERSION_MS = 750;

// --- защита от нестабильного питания в первые секунды после включения 220В ---
// При питании от AC-DC блока (в отличие от чистого USB с компьютера) в первые
// моменты после подачи 220В возможны просадки/шум на линии 3.3В (раскачка
// самого блока питания, одновременное срабатывание катушек ЭМР/подсветки
// и т.п.). Если именно в этот момент опросить датчик - можно получить
// ложный "не найден", и БЕЗ повторных попыток это остаётся так до перезагрузки.
const unsigned long POWER_STABILIZE_DELAY_MS = 1000;   // пауза перед первым обращением к датчикам
const uint8_t SENSOR_INIT_RETRY_COUNT = 5;             // сколько раз повторить поиск при неудаче
const unsigned long SENSOR_INIT_RETRY_DELAY_MS = 200;  // пауза между повторными попытками

// Пытается найти адрес датчика на шине с несколькими повторами (см. выше) -
// вместо однократной попытки, которая может ошибочно "потерять" датчик
// из-за кратковременной нестабильности питания именно в момент старта.
bool findSensorWithRetry(DallasTemperature &ds, DeviceAddress &addr) {
  for (uint8_t attempt = 0; attempt < SENSOR_INIT_RETRY_COUNT; attempt++) {
    if (ds.getAddress(addr, 0)) {
      return true;
    }
    delay(SENSOR_INIT_RETRY_DELAY_MS);
  }
  return false;
}

// Переинициализация одного датчика "на лету" (не при старте, а во время
// нормальной работы) - повторяет ту же процедуру, что и initSensors() для
// одного датчика: сброс шины (begin) + повторный поиск адреса + настройка
// разрешения и неблокирующего режима. Если физическая причина сбоя была
// временной (наводка, кратковременная просадка питания) - есть шанс
// восстановить связь без перезагрузки всего контроллера.
bool reinitSensor(DallasTemperature &ds, DeviceAddress &addr) {
  ds.begin();
  bool found = findSensorWithRetry(ds, addr);
  if (found) {
    ds.setResolution(addr, 12);
    ds.setWaitForConversion(false);
  }
  return found;
}

// Обрабатывает серию подряд идущих ошибок ОДНОГО датчика: при достижении
// SENSOR_ERROR_REINIT_THRESHOLD - пробует переинициализировать (см. выше),
// не больше SENSOR_REINIT_MAX_ATTEMPTS раз подряд. Счётчик серии сбрасывается
// после каждой попытки, чтобы отсчёт следующих 20 ошибок начинался заново.
// Возвращает true, если попытки уже ИСЧЕРПАНЫ и датчик всё ещё не отвечает -
// это сигнал вызывающему коду, что при желании можно поднимать аварию.
bool handleSensorErrorStreak(uint16_t &errorStreak, uint8_t &reinitAttempts,
                              DallasTemperature &ds, bool &foundFlag, DeviceAddress &addr) {
  if (errorStreak < SENSOR_ERROR_REINIT_THRESHOLD) {
    return false;
  }
  errorStreak = 0; // отсчёт следующей серии начинается заново

  if (reinitAttempts < SENSOR_REINIT_MAX_ATTEMPTS) {
    reinitAttempts++;
    diagSensorReinit((&foundFlag == &matFound) ? 0 : (&foundFlag == &airFound) ? 1 : 2);
    foundFlag = reinitSensor(ds, addr);
    return false; // попытка сделана, финальный вердикт пока не выносим
  }

  // все попытки уже израсходованы, а датчик по-прежнему не отвечает
  return true;
}

void initSensors() {
  // даём питанию 3.3В стабилизироваться после включения 220В, прежде чем
  // вообще что-либо спрашивать у датчиков
  delay(POWER_STABILIZE_DELAY_MS);

  // инициализация каждого датчика РАЗНЕСЕНА по времени (см. SENSOR_STAGGER_MS),
  // а не выполняется одновременно всех трёх сразу
  dsMat.begin();
  matFound = findSensorWithRetry(dsMat, addrMat);
  if (matFound) {
    dsMat.setResolution(addrMat, 12);      // 12 бит - максимальная точность DS18B20
    dsMat.setWaitForConversion(false);     // неблокирующий режим - опрос будет через millis()
  }
  delay(SENSOR_STAGGER_MS);

  dsAir.begin();
  airFound = findSensorWithRetry(dsAir, addrAir);
  if (airFound) {
    dsAir.setResolution(addrAir, 12);
    dsAir.setWaitForConversion(false);
  }
  delay(SENSOR_STAGGER_MS);

  dsOut.begin();
  outFound = findSensorWithRetry(dsOut, addrOut);
  if (outFound) {
    dsOut.setResolution(addrOut, 12);
    dsOut.setWaitForConversion(false);
  }
}

// ---------------------------------------------------------------------------
// ОПРОС ДАТЧИКОВ - неблокирующий, растянутый по времени state-machine.
// ---------------------------------------------------------------------------
// Раньше опрос трёх датчиков выполнялся одновременно (три requestTemperatures()
// подряд), что при БЛОКИРУЮЩЕМ режиме конверсии держало CPU занятым ~2.25с
// каждые 5 секунд. Теперь конверсия неблокирующая (setWaitForConversion(false)
// выставлен в initSensors()), и старт конверсии каждого датчика разнесён на
// SENSOR_STAGGER_MS, а чтение результата происходит только когда КОНКРЕТНО
// ЕГО конверсия гарантированно завершилась (DS18B20_CONVERSION_MS после
// старта именно этого датчика). Всё это не блокирует loop() ни на миллисекунду -
// энкодер, кнопка и проверка тока продолжают обрабатываться как обычно.

enum TempPollState : uint8_t {
  TPOLL_IDLE,             // цикл опроса не идёт, ждём следующего срабатывания по таймеру
  TPOLL_WAIT_AIR_START,   // Mat запущен, ждём момент старта Air
  TPOLL_WAIT_OUT_START,   // Air запущен, ждём момент старта Out
  TPOLL_WAIT_ALL_DONE     // Out запущен, ждём завершения его конверсии перед чтением всех трёх
};

TempPollState tempPollState = TPOLL_IDLE;
unsigned long tpMatStartMs = 0;
unsigned long tpAirStartMs = 0;
unsigned long tpOutStartMs = 0;

// Считывает результат конверсии со всех трёх датчиков (вызывается один раз
// в конце цикла опроса, когда все конверсии уже гарантированно завершены).
void readAllTemperatureResults() {
  if (matFound) {
    int16_t raw = dsMat.getTemp(addrMat);
    if (!isPlausibleRaw(raw)) {
      tempMatError = true; // старое значение tempMat НЕ трогаем
      diagSensorBad(0, raw);
      if (matErrorStreak < 0xFFFF) matErrorStreak++;
      matGoodStreak = 0;
      matOverheatCounter = 0; // подтверждение перегрева требует ПОДРЯД свежих чтений
    } else {
      tempMat = rawToTenths(raw);
      tempMatError = false;
      matErrorStreak = 0;
      matReinitAttempts = 0; // связь восстановлена - счётчик попыток тоже сбрасываем

      // автосброс "мягкой" аварии отказа датчика после нескольких хороших чтений
      if (matGoodStreak < 0xFF) matGoodStreak++;
      if (matGoodStreak >= ALARM_RECOVER_GOOD_COUNT) {
        alarmFlags &= ~ALARM_MAT_SENSOR_FAIL;
      }

      // перегрев коврика: нужно MAT_OVERHEAT_CONFIRM_COUNT свежих чтений подряд
      if (tempMat > MAT_TEMP_ALARM_TENTHS) {
        if (matOverheatCounter < 0xFF) matOverheatCounter++;
        if (matOverheatCounter >= MAT_OVERHEAT_CONFIRM_COUNT) {
          alarmFlags |= ALARM_MAT_OVERHEAT;
        }
      } else {
        matOverheatCounter = 0;
      }
    }
  } else {
    tempMatError = true; // датчик вообще не был найден при старте
    if (matErrorStreak < 0xFFFF) matErrorStreak++;
    matGoodStreak = 0;
    matOverheatCounter = 0;
  }
  // для коврика ЕСТЬ отдельная авария (см. ALARM_MAT_SENSOR_FAIL) - но она
  // не останавливает нагрев сама по себе, только предупреждает
  bool matReinitExhausted = handleSensorErrorStreak(matErrorStreak, matReinitAttempts, dsMat, matFound, addrMat);
  if (matReinitExhausted) {
    alarmFlags |= ALARM_MAT_SENSOR_FAIL;
  }

  if (airFound) {
    int16_t raw = dsAir.getTemp(addrAir);
    if (!isPlausibleRaw(raw)) {
      tempAirError = true;
      diagSensorBad(1, raw);
      if (airErrorStreak < 0xFFFF) airErrorStreak++;
      airGoodStreak = 0;
    } else {
      tempAir = rawToTenths(raw);
      tempAirError = false;
      tempAirLastGoodMs = millis();
      airErrorStreak = 0; // успешное чтение сбрасывает счётчик подряд идущих ошибок
      airReinitAttempts = 0;

      // автосброс аварии отказа опорного датчика после нескольких хороших
      // чтений подряд - ЭМР снова замкнётся, обычный термостат вернётся
      if (airGoodStreak < 0xFF) airGoodStreak++;
      if (airGoodStreak >= ALARM_RECOVER_GOOD_COUNT) {
        alarmFlags &= ~ALARM_AIR_SENSOR_FAIL;
      }
    }
  } else {
    tempAirError = true;
    if (airErrorStreak < 0xFFFF) airErrorStreak++;
    airGoodStreak = 0;
  }
  // для опорного датчика переинициализация - это ПОСЛЕДНИЙ ШАНС перед
  // аварией: авария поднимается только когда попытки уже исчерпаны
  bool airReinitExhausted = handleSensorErrorStreak(airErrorStreak, airReinitAttempts, dsAir, airFound, addrAir);
  if (airReinitExhausted) {
    alarmFlags |= ALARM_AIR_SENSOR_FAIL;
  }

  if (outFound) {
    int16_t raw = dsOut.getTemp(addrOut);
    if (!isPlausibleRaw(raw)) {
      tempOutError = true;
      diagSensorBad(2, raw);
      if (outErrorStreak < 0xFFFF) outErrorStreak++;
    } else {
      tempOut = rawToTenths(raw);
      tempOutError = false;
      outErrorStreak = 0;
      outReinitAttempts = 0;
    }
  } else {
    tempOutError = true;
    if (outErrorStreak < 0xFFFF) outErrorStreak++;
  }
  handleSensorErrorStreak(outErrorStreak, outReinitAttempts, dsOut, outFound, addrOut);

  tempPollCompletedOnce = true;
}

// Запускает новый цикл опроса (вызывается из loop() по таймеру TEMP_POLL_INTERVAL_MS,
// либо один раз сразу при старте из setup()).
void startTempPollCycle() {
  if (matFound) dsMat.requestTemperatures();
  tpMatStartMs = millis();
  tempPollState = TPOLL_WAIT_AIR_START;
}

// Продвигает state-machine опроса - вызывается КАЖДУЮ итерацию loop(),
// сама решает, пора ли переходить к следующему шагу (или ничего не делает).
void updateTempPollStateMachine() {
  unsigned long now = millis();

  switch (tempPollState) {
    case TPOLL_IDLE:
      break; // ждём внешнего вызова startTempPollCycle()

    case TPOLL_WAIT_AIR_START:
      if (now - tpMatStartMs >= SENSOR_STAGGER_MS) {
        if (airFound) dsAir.requestTemperatures();
        tpAirStartMs = now;
        tempPollState = TPOLL_WAIT_OUT_START;
      }
      break;

    case TPOLL_WAIT_OUT_START:
      if (now - tpAirStartMs >= SENSOR_STAGGER_MS) {
        if (outFound) dsOut.requestTemperatures();
        tpOutStartMs = now;
        tempPollState = TPOLL_WAIT_ALL_DONE;
      }
      break;

    case TPOLL_WAIT_ALL_DONE:
      // ждём гарантированного завершения САМОЙ ПОЗДНЕЙ конверсии (Out) -
      // к этому моменту Mat и Air уже точно готовы (стартовали раньше)
      if (now - tpOutStartMs >= DS18B20_CONVERSION_MS) {
        readAllTemperatureResults();
        tempPollState = TPOLL_IDLE;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// ТЕРМОСТАТ (гистерезис по опорному датчику "воздух")
// ---------------------------------------------------------------------------
void updateThermostat() {
  bool newSsrState = ssrState; // по умолчанию - без изменений (гистерезис ничего не решил)

  // "жёсткие" аварии - реальные электрические/тепловые неисправности,
  // при которых нагрев останавливается БЕЗ каких-либо исключений, включая
  // аварийный таймерный режим ниже. ALARM_MAT_SENSOR_FAIL сюда намеренно
  // не входит - отказ ОДНОГО датчика коврика при исправном опорном датчике
  // не повод останавливать нагрев (см. обсуждение), только предупреждение.
  bool hardStopAlarm = alarmFlags & (ALARM_MAT_OVERHEAT | ALARM_OVERCURRENT | ALARM_SSR_STUCK);

  if (!ssrStartupDelayPassed) {
    // первые SSR_STARTUP_DELAY_MS после старта контроллера нагрев запрещён
    // в принципе, независимо от показаний опорного датчика - см. константу
    newSsrState = false;
    emergencyModeActive = false;
    emergencyTimerInitialized = false;
  } else if (hardStopAlarm) {
    // жёсткая авария - нагрев выключен безусловно, аварийный режим не спасает
    newSsrState = false;
    emergencyModeActive = false;
    emergencyTimerInitialized = false;
  } else if (tempAir == -1270) {
    // данных с опорного датчика не было вообще ни разу с момента старта -
    // управлять нечем (ALARM_AIR_SENSOR_FAIL для этого случая ещё не успеет
    // сработать в первые ~секунды/минуты после старта - см. её порог)
    newSsrState = false;
    emergencyModeActive = false;
    emergencyTimerInitialized = false;
  } else if (alarmFlags & ALARM_AIR_SENSOR_FAIL) {
    // опорный датчик отказал (после исчерпания переинициализаций) - обычный
    // термостат управлять не может. Если на улице мороз - переходим на
    // грубый аварийный таймер вместо полной остановки нагрева.
    if (emergencyModeEligible()) {
      emergencyModeActive = true;
      newSsrState = updateEmergencyTimer();
    } else {
      // либо данных с улицы нет, либо там не холодно - оставлять нагрев
      // включённым вслепую не оправдано, глушим до восстановления датчика
      emergencyModeActive = false;
      emergencyTimerInitialized = false;
      newSsrState = false;
    }
  } else if (millis() - tempAirLastGoodMs > AIR_DATA_STALE_MS) {
    // авария датчика ещё не поднята (идут переинициализации), но свежих
    // данных уже давно нет - по устаревшему значению гистерезис не применяем,
    // нагрев выключается. Если связь восстановится, управление вернётся само.
    emergencyModeActive = false;
    emergencyTimerInitialized = false;
    newSsrState = false;
  } else {
    // штатный режим - обычный гистерезис по опорному датчику
    emergencyModeActive = false;
    emergencyTimerInitialized = false;

    int16_t setpointTenths = (int16_t)setpointC * 10;
    if (!ssrState && tempAir < (setpointTenths - HYSTERESIS_TENTHS)) {
      newSsrState = true;
    } else if (ssrState && tempAir > (setpointTenths + HYSTERESIS_TENTHS)) {
      newSsrState = false;
    }
  }

  if (newSsrState != ssrState) {
    ssrState = newSsrState;
    digitalWrite(PIN_SSR, ssrState ? SSR_ACTIVE_LEVEL : SSR_INACTIVE_LEVEL);

    // при ЛЮБОМ переключении SSR (включении или выключении) открываем
    // "слепое окно" для датчика тока - переходные процессы в момент
    // коммутации не должны восприниматься как авария (см. checkCurrent())
    lastSsrTransitionMs = millis();
    overcurrentCounter = 0;
    ssrStuckCounter = 0;
    openLoadCounter = 0;
  }
}

// Аварийная защита по температуре коврика (ALARM_MAT_OVERHEAT) проверяется
// прямо в readAllTemperatureResults() - только по СВЕЖИМ чтениям, с
// подтверждением несколькими подряд (см. MAT_OVERHEAT_CONFIRM_COUNT).

// ---------------------------------------------------------------------------
// АВАРИЙНАЯ ЗАЩИТА ПО ТОКУ (КЗ / обрыв / залипание SSR)
// ---------------------------------------------------------------------------

// Целочисленный квадратный корень (метод Ньютона) - без float, быстро сходится.
int32_t integerSqrt(int32_t value) {
  if (value <= 0) return 0;
  int32_t x = value;
  int32_t y = (x + 1) / 2;
  while (y < x) {
    x = y;
    y = (x + value / x) / 2;
  }
  return x;
}

// Считает RMS переменной составляющей сигнала с АЦП за одно окно ~20мс
// (один период сети 50Гц) - в отличие от peak-to-peak, использует ВСЕ
// отсчёты окна, а не только минимум/максимум, поэтому существенно менее
// чувствителен к случайному шуму отдельных выборок.
//
// RMS считается через дисперсию: rms = sqrt(mean(x^2) - mean(x)^2).
// Среднее (mean) берётся из ТОГО ЖЕ окна, поэтому не нужно заранее знать
// точный уровень смещения (bias) схемы - он вычитается автоматически,
// в том числе если немного "плавает" от температуры платы.
//
// Блокирующая функция (сама выборка занимает ~20мс) - вызывается как один
// "под-замер" внутри внешнего цикла усреднения, см. checkCurrent().
int measureCurrentRms() {
  int32_t count = 0;
  int32_t sum = 0;
  int64_t sumSquares = 0;
  unsigned long start = micros();

  while ((micros() - start) < CURRENT_SAMPLE_WINDOW_US) {
    int32_t v = analogRead(PIN_CURRENT);
    sum += v;
    sumSquares += (int64_t)v * (int64_t)v;
    count++;
  }

  if (count == 0) return 0;

  int32_t mean = sum / count;
  int64_t meanSquare = sumSquares / count;
  int32_t variance = (int32_t)(meanSquare - (int64_t)mean * (int64_t)mean);

  return (int)integerSqrt(variance);
}

unsigned long lastCurrentCheckMs = 0;

// измеренный на старте (пока SSR гарантированно выключен) собственный шум
// платы датчика тока, в СЫРЫХ единицах АЦП - см. measureCurrentNoiseBaseline().
// Используется для вычитания шумового пола в квадратуре в evaluateAveragedCurrent().
int32_t currentNoiseBaselineRms = 0;
bool currentBaselineCalibrated = false;

// Измеряет собственный шумовой уровень датчика тока при заведомо нулевом
// токе (SSR выключен) - усредняет несколько выборок RMS. Вызывается
// ОДИН РАЗ в setup(), пока SSR ещё не может быть включён (см. SSR_STARTUP_DELAY_MS).
void measureCurrentNoiseBaseline() {
  const uint8_t SAMPLES = 8;
  int32_t sumRms = 0;

  for (uint8_t i = 0; i < SAMPLES; i++) {
    sumRms += measureCurrentRms();
  }

  currentNoiseBaselineRms = sumRms / SAMPLES;
  currentBaselineCalibrated = true;
}

// --- усреднение показаний датчика тока ---
// Аналоговый датчик даёт заметный разброс между отдельными измерениями
// peak-to-peak даже при стабильном токе. Вместо одного замера на цикл
// проверки теперь берётся CURRENT_AVG_SAMPLE_COUNT замеров с интервалом
// CURRENT_AVG_SAMPLE_INTERVAL_MS между ними, и используется их среднее.
// Реализовано как неблокирующая машина состояний (растянута по времени,
// как и опрос датчиков температуры) - иначе набор 10 замеров с паузами
// занял бы ~1 секунду СПЛОШНОГО блокирования loop() на каждую проверку.
const uint8_t CURRENT_AVG_SAMPLE_COUNT = 10;
const unsigned long CURRENT_AVG_SAMPLE_INTERVAL_MS = 100;

enum CurrentCheckState : uint8_t {
  CCHECK_IDLE,      // ждём начала нового цикла усреднения (по таймеру CURRENT_CHECK_INTERVAL_MS)
  CCHECK_SAMPLING   // цикл идёт, собираем очередные выборки
};

CurrentCheckState currentCheckState = CCHECK_IDLE;
uint8_t currentSampleIndex = 0;
int32_t currentSampleSumRms = 0;
unsigned long currentSampleLastMs = 0;

// Вызывается, когда набор из CURRENT_AVG_SAMPLE_COUNT выборок усреднён -
// вся прежняя логика перевода в амперы и проверки аварий, без изменений
// по сути, просто теперь получает на вход уже усреднённый RMS.
void evaluateAveragedCurrent(int rms) {
  // --- вычитаем собственный шумовой пол датчика (в квадратуре, т.к. RMS
  // независимых источников складываются геометрически: rms_общ² = rms_сигнал² + rms_шум²,
  // поэтому и вычитать нужно так же, а не простой разностью) ---
  int32_t adjustedRms = rms;
  if (currentBaselineCalibrated) {
    int64_t rmsSquared = (int64_t)rms * (int64_t)rms;
    int64_t baselineSquared = (int64_t)currentNoiseBaselineRms * (int64_t)currentNoiseBaselineRms;
    int64_t diffSquared = rmsSquared - baselineSquared;
    if (diffSquared < 0) diffSquared = 0; // шум в моменте оказался чуть выше усреднённой базовой линии - это нормально
    adjustedRms = integerSqrt((int32_t)diffSquared);
  }

  // --- пересчёт в реальный ток (используется и для отображения, и для
  // самой аварийной логики ниже - пороги заданы в реальных амперах) ---
  int32_t currentMilliamps = (adjustedRms * CURRENT_CALIB_MA_AT_RMS) / CURRENT_CALIB_RMS_VALUE;
  currentAmpsHundredths = (int16_t)(currentMilliamps / 10); // мА -> сотые доли ампера

  // "слепое окно" после переключения SSR - измерение и отображение тока
  // продолжаются как обычно (см. выше), а вот АВАРИЙНАЯ ЛОГИКА по этому
  // датчику на время переходного процесса приостанавливается
  if (millis() - lastSsrTransitionMs < CURRENT_CHECK_BLANKING_MS) {
    return;
  }

  if (ssrState) {
    // SSR должен быть включен: проверяем превышение тока и обрыв нагрузки
    if (currentAmpsHundredths > CURRENT_OVERCURRENT_HUNDREDTHS) {
      overcurrentCounter++;
      if (overcurrentCounter >= CURRENT_FAULT_CONFIRM_COUNT) {
        alarmFlags |= ALARM_OVERCURRENT;
      }
    } else {
      overcurrentCounter = 0;
    }

    if (currentAmpsHundredths < CURRENT_OPEN_LOAD_HUNDREDTHS) {
      openLoadGoodCounter = 0;
      openLoadCounter++;
      if (openLoadCounter >= CURRENT_FAULT_CONFIRM_COUNT) {
        alarmFlags |= ALARM_OPEN_LOAD; // не разрываем ЭМР - не опасно, просто сигнал
      }
    } else {
      openLoadCounter = 0;
      // ток снова в норме - после нескольких таких проверок подряд аварию
      // обрыва нагрузки снимаем автоматически (она ничего не размыкает)
      if (alarmFlags & ALARM_OPEN_LOAD) {
        openLoadGoodCounter++;
        if (openLoadGoodCounter >= ALARM_RECOVER_GOOD_COUNT) {
          alarmFlags &= ~ALARM_OPEN_LOAD;
          openLoadGoodCounter = 0;
        }
      }
    }
  } else {
    // SSR должен быть выключен: проверяем, что тока действительно нет (не залип).
    // currentAmpsHundredths уже очищен от базовой линии шума (см. выше в этой
    // же функции - вычитание в квадратуре), поэтому порог здесь - это просто
    // запас на разброс шума вокруг усреднённой базовой линии, а не сама
    // базовая линия ещё раз. Пока базовая линия не измерена (маловероятно,
    // но на всякий случай) - проверку по этому пункту пропускаем.
    if (currentBaselineCalibrated) {
      if (currentAmpsHundredths > CURRENT_LEAKAGE_MARGIN_HUNDREDTHS) {
        ssrStuckCounter++;
        if (ssrStuckCounter >= CURRENT_FAULT_CONFIRM_COUNT) {
          alarmFlags |= ALARM_SSR_STUCK;
        }
      } else {
        ssrStuckCounter = 0;
      }
    }
  }
}

void checkCurrent() {
  unsigned long now = millis();

  if (currentCheckState == CCHECK_IDLE) {
    if (now - lastCurrentCheckMs < CURRENT_CHECK_INTERVAL_MS) {
      return; // ещё не время начинать новый цикл усреднения
    }
    lastCurrentCheckMs = now;
    currentCheckState = CCHECK_SAMPLING;
    currentSampleIndex = 0;
    currentSampleSumRms = 0;
    // "минус интервал", чтобы самая первая выборка бралась немедленно,
    // не дожидаясь ещё 100мс сверху
    currentSampleLastMs = now - CURRENT_AVG_SAMPLE_INTERVAL_MS;
  }

  if (currentCheckState == CCHECK_SAMPLING) {
    if (now - currentSampleLastMs >= CURRENT_AVG_SAMPLE_INTERVAL_MS) {
      currentSampleLastMs = now;
      currentSampleSumRms += measureCurrentRms(); // блокирует на ~20мс - незначительно
      currentSampleIndex++;

      if (currentSampleIndex >= CURRENT_AVG_SAMPLE_COUNT) {
        int rms = (int)(currentSampleSumRms / CURRENT_AVG_SAMPLE_COUNT);
        currentCheckState = CCHECK_IDLE;
        evaluateAveragedCurrent(rms);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// УПРАВЛЕНИЕ ЭМР (fail-safe разрыв цепи при аварии)
// ---------------------------------------------------------------------------
void updateEmr() {
  // ЭМР держим замкнутым, пока нет аварий, требующих физического разрыва,
  // И система уже готова (прошла задержка после старта - см. systemReady).
  // ALARM_OPEN_LOAD не требует разрыва (не опасно - просто нет нагрузки/связи).
  // ALARM_MAT_SENSOR_FAIL тоже не требует - см. её описание в enum AlarmBits.
  // ALARM_AIR_SENSOR_FAIL требует разрыва, ЗА ИСКЛЮЧЕНИЕМ случая, когда
  // активен аварийный таймерный режим обогрева (emergencyModeActive) - иначе
  // ЭМР (которая стоит ДО SSR в цепи) обесточила бы нагреватель полностью,
  // и аварийный таймер физически не смог бы ничего греть.
  bool shouldBeClosed = systemReady &&
      !(alarmFlags & (ALARM_MAT_OVERHEAT | ALARM_OVERCURRENT | ALARM_SSR_STUCK)) &&
      (!(alarmFlags & ALARM_AIR_SENSOR_FAIL) || emergencyModeActive);

  if (shouldBeClosed != emrClosed) {
    emrClosed = shouldBeClosed;
    digitalWrite(PIN_EMR, emrClosed ? EMR_ENERGIZED_LEVEL : EMR_DEENERGIZED_LEVEL);
  }
}

// ---------------------------------------------------------------------------
// ОБРАБОТКА ЭНКОДЕРА (перевод импульсов в изменение уставки)
// ---------------------------------------------------------------------------
void handleEncoder() {
  noInterrupts();
  int32_t delta = encoderRawDelta;
  encoderRawDelta = 0;
  interrupts();

  if (!backlightOn) {
    // при выключенной подсветке вращение энкодера полностью игнорируется -
    // накопленные импульсы просто отбрасываются (не копятся "про запас"
    // до включения подсветки), меняется только состояние самой подсветки
    // через нажатие кнопки (см. onShortPress()/onLongPress())
    return;
  }

  if (delta == 0) return;

  int32_t clicks = delta / ENCODER_STEPS_PER_CLICK;
  if (clicks == 0) {
    // накопленных импульсов не хватает на целый "клик" - возвращаем остаток обратно
    noInterrupts();
    encoderRawDelta += delta;
    interrupts();
    return;
  }

  lastEncoderActivityMs = millis(); // любое вращение - активность (во всех режимах)

  if (systemMode == MODE_WIFI_SETUP_CONFIRM) {
    // здесь вращение просто переключает выбор ДА/НЕТ - направление
    // роли не играет, любой клик меняет текущий выбор на противоположный
    wifiSetupConfirmYes = !wifiSetupConfirmYes;
    wifiSetupConfirmActivityMs = millis();
    return;
  }

  if (systemMode != MODE_NORMAL) {
    // в активном портале настройки (MODE_WIFI_SETUP_ACTIVE) вращение
    // энкодера пока не используется
    return;
  }

  if (!editingSetpoint) {
    editingSetpoint = true;
    pendingSetpointC = setpointC;
  }
  editingActivityMs = millis(); // любое вращение (вход в режим или изменение) сбрасывает таймаут

  int32_t newVal = (int32_t)pendingSetpointC + clicks;
  if (newVal < SETPOINT_MIN_C) newVal = SETPOINT_MIN_C;
  if (newVal > SETPOINT_MAX_C) newVal = SETPOINT_MAX_C;
  pendingSetpointC = (int8_t)newVal;
}

// Если редактирование уставки энкодером не подтверждено коротким нажатием
// в течение ENCODER_EDIT_TIMEOUT_MS - отменяем его, возвращая последнюю
// подтверждённую уставку (визуально это просто выход из режима редактирования,
// т.к. отображение при !editingSetpoint и так показывает setpointC).
void checkEncoderEditTimeout() {
  if (editingSetpoint && (millis() - editingActivityMs >= ENCODER_EDIT_TIMEOUT_MS)) {
    editingSetpoint = false;
    pendingSetpointC = setpointC;
  }
}

// Если экран подтверждения входа в настройку Wi-Fi долго не получает
// решения - отменяем его и возвращаемся к обычной работе (безопаснее, чем
// надолго "зависать" в административном экране без присмотра).
void checkWifiSetupConfirmTimeout() {
  if (systemMode == MODE_WIFI_SETUP_CONFIRM &&
      (millis() - wifiSetupConfirmActivityMs >= WIFI_SETUP_CONFIRM_TIMEOUT_MS)) {
    systemMode = MODE_NORMAL;
  }
}

// Автоматически выключает подсветку, если не было ни вращения энкодера,
// ни нажатий кнопки в течение BACKLIGHT_AUTO_OFF_MS. НЕ срабатывает, пока
// идёт настройка Wi-Fi (см. systemMode) - гасить экран посреди активного
// взаимодействия с меню настройки не нужно.
void checkBacklightAutoOff() {
  if (systemMode != MODE_NORMAL) return;

  if (backlightOn && (millis() - lastEncoderActivityMs >= BACKLIGHT_AUTO_OFF_MS)) {
    backlightOn = false;
    digitalWrite(PIN_TFT_LED, LOW);
  }
}

// ---------------------------------------------------------------------------
// ЭКРАН
// ---------------------------------------------------------------------------

// Печатает значение температуры (в десятых долях) в формате "XX.X" в
// заданной точке, предварительно затирая прямоугольник под старым текстом.
// Если hasError == true - рядом с числом (в фиксированной позиции)
// дополнительно рисуется жёлтый "?", сигнализирующий, что это значение
// могло устареть (последнее чтение с датчика не удалось).
void drawTempField(int x, int y, int w, int h, int16_t tenths, uint16_t color, bool hasError) {
  tft.fillRect(x, y, w, h, TFT_BLACK);
  tft.setCursor(x, y);
  tft.setTextColor(color, TFT_BLACK);

  if (tenths == -1270) {
    tft.print("--.-"); // данных не было вообще ни разу
  } else {
    int16_t whole = tenths / 10;
    int16_t frac = tenths % 10;
    if (frac < 0) frac = -frac;

    tft.print(whole);
    tft.print(".");
    tft.print(frac);
  }

  if (hasError) {
    // знак вопроса рисуется в ФИКСИРОВАННОЙ позиции справа от числа,
    // независимо от длины самого числа - чтобы не "прыгал" при смене
    // количества цифр (например "9.5" короче, чем "-15.5")
    tft.setCursor(x + 95, y);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print("?");
  }
}

void drawStaticLabels() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2); // подписи чуть мельче, чтобы дать место под "?" у значений

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 18);  tft.print("OUTDOOR");
  tft.setCursor(10, 54);  tft.print("INDOOR");
  tft.setCursor(10, 90);  tft.print("BOTTOM");
  tft.setCursor(10, 126); tft.print("SET TO");
  tft.setCursor(10, 184); tft.print("CURRENT");
}

// Строка статуса Wi-Fi + реального времени занимает самый верх экрана
// (size 1 - это служебная, не основная информация, крупный шрифт ей не
// нужен). Перерисовывается только при реальном изменении содержимого
// (смена статуса, IP или минуты текущего времени).
String lastShownWifiLine = "";
unsigned long lastWifiStatusCheckMs = 0;

void drawWifiStatus() {
  // Строка собирается из String'ов (IP, время) - делать это на каждой
  // итерации loop() (тысячи раз в секунду) незачем: пересчитываем раз в
  // секунду. Сброс трекера (lastShownWifiLine == "", см. forceNormalScreenRedraw())
  // форсирует немедленную отрисовку без ожидания.
  unsigned long nowMs = millis();
  if (lastShownWifiLine.length() > 0 && (nowMs - lastWifiStatusCheckMs) < 1000) {
    return;
  }
  lastWifiStatusCheckMs = nowMs;

  String wifiPart;
  if (wifiState == WIFI_ST_CONNECTED) {
    wifiPart = "WiFi: OK  " + WiFi.localIP().toString();
  } else {
    wifiPart = "WiFi: podklyuchenie...";
  }

  String timePart = "";
  if (timeSynced) {
    time_t nowEpoch = time(nullptr);
    struct tm timeinfo;
    localtime_r(&nowEpoch, &timeinfo);
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    timePart = String(buf);
  }

  String fullLine = wifiPart + "  " + timePart; // для определения "изменилось ли что-то"

  if (fullLine != lastShownWifiLine) {
    tft.setTextSize(1);
    tft.fillRect(0, 0, 240, 10, TFT_BLACK);
    tft.setCursor(2, 2);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print(wifiPart);

    if (timePart.length() > 0) {
      tft.print("  ");
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.print(timePart);
    }

    lastShownWifiLine = fullLine;
  }
}

// ---------------------------------------------------------------------------
// ЭКРАНЫ РЕЖИМОВ НАСТРОЙКИ WI-FI
// ---------------------------------------------------------------------------
SystemMode lastRenderedMode = MODE_NORMAL; // какой экран физически нарисован сейчас

// --- Стартовый экран с версией прошивки ---
// Показывается сразу после инициализации дисплея (см. setup()) и держится
// ровно VERSION_SPLASH_DURATION_MS, после чего updateDisplay() один раз
// затирает его и переходит к обычному экрану с показаниями - без каких-либо
// иных изменений в дальнейшей логике отрисовки.
const unsigned long VERSION_SPLASH_DURATION_MS = 5000;
bool showingVersionSplash = true;
unsigned long versionSplashStartMs = 0;

// При возврате в MODE_NORMAL из любого другого режима экран содержит чужой
// контент (экран подтверждения / экран портала) - обычная логика "перерисовать
// только если изменилось" сама по себе этого не заметит, т.к. сравнивает с
// тем, что БЫЛО ПОСЛЕДНИЙ РАЗ НАРИСОВАНО В NORMAL-РЕЖИМЕ, а не с тем, что
// физически сейчас на экране. Поэтому здесь принудительно сбрасываем все
// трекеры "последнего показанного значения" - это форсирует полную
// перерисовку на ближайшем вызове обычной части updateDisplay().
void forceNormalScreenRedraw() {
  drawStaticLabels();

  tempOutLastShown = 32000;
  tempAirLastShown = 32000;
  tempMatLastShown = 32000;
  tempOutErrorLastShown = !tempOutError; // булевы трекеры сбрасываем в ПРОТИВОПОЛОЖНОЕ
  tempAirErrorLastShown = !tempAirError; // текущему состояние - гарантированное
  tempMatErrorLastShown = !tempMatError; // несовпадение при следующем сравнении

  lastShownSetpointValid = false;

  lastShownSsrState = -1;
  lastShownEmergencyMode = !emergencyModeActive;

  currentAmpsLastShown = 32000;

  lastShownAlarmFlags = 0xFFFF;

  lastShownWifiLine = "";
}

// Экран "Войти в настройку Wi-Fi? ДА/НЕТ" - вращение энкодера переключает
// выбор (см. handleEncoder()), короткое нажатие подтверждает (см. onShortPress()).
bool lastShownWifiConfirmYes = false;

void drawWifiConfirmScreen() {
  bool modeJustEntered = (lastRenderedMode != MODE_WIFI_SETUP_CONFIRM);

  if (modeJustEntered) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 40);
    tft.print("Voyti v nastroyku");
    tft.setCursor(10, 65);
    tft.print("Wi-Fi?");

    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 280);
    tft.print("Vrashenie - vybor, korotkoe nazhatie - OK");

    lastRenderedMode = MODE_WIFI_SETUP_CONFIRM;
    lastShownWifiConfirmYes = !wifiSetupConfirmYes; // форсируем перерисовку блока выбора ниже
  }

  if (wifiSetupConfirmYes != lastShownWifiConfirmYes) {
    tft.setTextSize(3);
    tft.fillRect(10, 120, 220, 30, TFT_BLACK);
    tft.setCursor(10, 120);
    tft.setTextColor(wifiSetupConfirmYes ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    tft.print(wifiSetupConfirmYes ? "> DA" : "  DA");

    tft.fillRect(10, 160, 220, 30, TFT_BLACK);
    tft.setCursor(10, 160);
    tft.setTextColor(!wifiSetupConfirmYes ? TFT_RED : TFT_DARKGREY, TFT_BLACK);
    tft.print(!wifiSetupConfirmYes ? "> NET" : "  NET");

    lastShownWifiConfirmYes = wifiSetupConfirmYes;
  }
}

// Экран активного портала настройки - статические инструкции, рисуются
// один раз при входе в режим, дальше ничего на этом экране не меняется
// (кроме как через выход из режима целиком).
void drawWifiSetupActiveScreen() {
  if (lastRenderedMode == MODE_WIFI_SETUP_ACTIVE) return; // уже нарисован, менять нечего

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 20);
  tft.print("NASTROYKA WI-FI");

  tft.setCursor(10, 60);
  tft.print("Podklyuchites k seti:");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 84);
  tft.print("Budka-Setup");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 120);
  tft.print("Otkroyte v brauzere:");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 144);
  tft.print("192.168.4.1");

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, 280);
  tft.print("Dolgoe nazhatie - otmena nastroyki");

  lastRenderedMode = MODE_WIFI_SETUP_ACTIVE;
}

void updateDisplay() {
  // Стартовый экран версии прошивки - держим его ровно
  // VERSION_SPLASH_DURATION_MS с момента инициализации дисплея (см. setup()),
  // ничего больше в это время не перерисовываем. По истечении времени -
  // один раз затираем экран и переходим к обычной отрисовке (ниже) без
  // каких-либо иных изменений в её поведении.
  if (showingVersionSplash) {
    if (millis() - versionSplashStartMs < VERSION_SPLASH_DURATION_MS) {
      return;
    }
    showingVersionSplash = false;
    forceNormalScreenRedraw(); // затирает сплэш (рисует статичные подписи) и сбрасывает трекеры
  }

  if (systemMode == MODE_WIFI_SETUP_CONFIRM) {
    drawWifiConfirmScreen();
    return;
  }
  if (systemMode == MODE_WIFI_SETUP_ACTIVE) {
    drawWifiSetupActiveScreen();
    return;
  }

  // MODE_NORMAL - если только что вернулись из другого режима, экран
  // сейчас содержит чужой контент - форсируем полную перерисовку
  if (lastRenderedMode != MODE_NORMAL) {
    forceNormalScreenRedraw();
  }
  lastRenderedMode = MODE_NORMAL;

  drawWifiStatus();

  // --- основные показания: крупный шрифт (size 3) ---
  tft.setTextSize(3);

  // порядок: улица (синий) -> воздух в будке (белый) -> коврик (красный)
  if (tempOut != tempOutLastShown || tempOutError != tempOutErrorLastShown) {
    drawTempField(100, 18, 130, 28, tempOut, TFT_BLUE, tempOutError);
    tempOutLastShown = tempOut;
    tempOutErrorLastShown = tempOutError;
  }
  if (tempAir != tempAirLastShown || tempAirError != tempAirErrorLastShown) {
    drawTempField(100, 54, 130, 28, tempAir, TFT_WHITE, tempAirError);
    tempAirLastShown = tempAir;
    tempAirErrorLastShown = tempAirError;
  }
  if (tempMat != tempMatLastShown || tempMatError != tempMatErrorLastShown) {
    drawTempField(100, 90, 130, 28, tempMat, TFT_RED, tempMatError);
    tempMatLastShown = tempMat;
    tempMatErrorLastShown = tempMatError;
  }

  // --- уставка / режим редактирования ---
  // текущее отображаемое значение зависит от режима: в редактировании
  // показываем pendingSetpointC, иначе - активную setpointC
  int8_t currentValueToShow = editingSetpoint ? pendingSetpointC : setpointC;

  bool setpointNeedsRedraw =
      (!lastShownSetpointValid) ||
      (currentValueToShow != lastShownSetpointValue) ||
      (editingSetpoint != lastShownEditingMode);

  if (setpointNeedsRedraw) {
    tft.fillRect(100, 126, 130, 28, TFT_BLACK);
    tft.setCursor(100, 126);
    tft.setTextColor(editingSetpoint ? TFT_YELLOW : TFT_GREEN, TFT_BLACK);
    tft.print(currentValueToShow);
    tft.print(editingSetpoint ? " ?" : " C");

    lastShownSetpointValue = currentValueToShow;
    lastShownEditingMode = editingSetpoint;
    lastShownSetpointValid = true;
  }

  // --- статус нагрева и аварии: шрифт поменьше (size 2), т.к. текстовые
  // сообщения об авариях длиннее и должны помещаться по ширине экрана ---
  tft.setTextSize(2);

  // --- статус нагрева (легенда всегда белым, меняется только текст).
  // Перерисовывается и при смене состояния SSR, и при входе/выходе из
  // аварийного таймерного режима - иначе индикатор "EM!" не появится
  // вовремя, если сам ssrState в этот момент не изменился. ---
  int8_t ssrStateAsInt = ssrState ? 1 : 0;
  if (ssrStateAsInt != lastShownSsrState || emergencyModeActive != lastShownEmergencyMode) {
    tft.fillRect(10, 162, 220, 20, TFT_BLACK);
    tft.setCursor(10, 162);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(ssrState ? "HEATING: ON" : "HEATING: OFF");
    if (emergencyModeActive) {
      tft.print(" EM!");
    }

    lastShownSsrState = ssrStateAsInt;
    lastShownEmergencyMode = emergencyModeActive;
  }

  // --- ток через нагреватель, отдельная строка (для калибровки датчика
  // тока и наблюдения за деградацией ИК-плёнки со временем), формат "0.41A" ---
  if (currentAmpsForDisplay != currentAmpsLastShown) {
    tft.fillRect(100, 184, 90, 20, TFT_BLACK);
    tft.setCursor(100, 184);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    int16_t ampsWhole = currentAmpsForDisplay / 100;
    int16_t ampsFrac  = currentAmpsForDisplay % 100;
    if (ampsFrac < 0) ampsFrac = -ampsFrac;

    tft.print(ampsWhole);
    tft.print(".");
    if (ampsFrac < 10) tft.print("0"); // ведущий ноль, чтобы всегда было 2 знака после запятой
    tft.print(ampsFrac);
    tft.print("A");

    currentAmpsLastShown = currentAmpsForDisplay;
  }

  // --- аварии (легенда всегда жёлтым, каждая авария на своей строке,
  // чтобы не вылезать за ширину экрана при нескольких одновременных) ---
  if (alarmFlags != lastShownAlarmFlags) {
    tft.fillRect(10, 208, 220, 100, TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);

    if (alarmFlags == ALARM_NONE) {
      tft.setCursor(10, 208);
      tft.print("ALARM: NONE");
    } else {
      int line = 0;
      if (alarmFlags & ALARM_MAT_OVERHEAT) {
        tft.setCursor(10, 208 + line * 20); tft.print("PEREGREV KOVRIKA!"); line++;
      }
      if (alarmFlags & ALARM_OVERCURRENT) {
        tft.setCursor(10, 208 + line * 20); tft.print("PREVYSHEN TOK!"); line++;
      }
      if (alarmFlags & ALARM_SSR_STUCK) {
        tft.setCursor(10, 208 + line * 20); tft.print("SSR ZALIP!"); line++;
      }
      if (alarmFlags & ALARM_OPEN_LOAD) {
        tft.setCursor(10, 208 + line * 20); tft.print("OBRYV NAGRUZKI!"); line++;
      }
      if (alarmFlags & ALARM_AIR_SENSOR_FAIL) {
        tft.setCursor(10, 208 + line * 20); tft.print("OTKAZ DATCHIKA VOZDUHA!"); line++;
      }
      if (alarmFlags & ALARM_MAT_SENSOR_FAIL) {
        tft.setCursor(10, 208 + line * 20); tft.print("OTKAZ DATCHIKA KOVRIKA"); line++;
      }
    }

    lastShownAlarmFlags = alarmFlags;
  }
}

// ---------------------------------------------------------------------------
// WI-FI - неблокирующее подключение
// ---------------------------------------------------------------------------
// WiFi.begin() сам по себе не блокирует - подключение идёт в фоне, здесь
// только проверяется его текущий статус (WiFi.status()) через millis(),
// без единого delay(). Если подключиться не удалось за отведённое время -
// пробуем заново (не сдаёмся навсегда, вдруг роутер был временно недоступен).

// Запускает подключение к Wi-Fi по данным, сохранённым ESP32 во внутренней
// NVS (WiFi.begin() без аргументов) - либо от предыдущего успешного сеанса
// портала настройки, либо пусто, если сеть ещё ни разу не настраивалась.
// Никаких зашитых в прошивку учётных данных как резервного варианта нет -
// на "чистом" контроллере просто ничего не подключится, пока пользователь
// не настроит сеть вручную через портал (длинное нажатие энкодера).
void connectToWifi() {
  WiFi.begin();
}

void startWifiConnection() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME); // ВАЖНО: до WiFi.begin(), иначе не применится
  connectToWifi();
  wifiState = WIFI_ST_CONNECTING;
  wifiStateChangeMs = millis();
}

void updateWifiConnection() {
  unsigned long now = millis();

  if (wifiState == WIFI_ST_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiState = WIFI_ST_CONNECTED;
      wifiStateChangeMs = now;

      // веб-сервер поднимаем ТОЛЬКО теперь, когда Wi-Fi реально подключен -
      // без сети его всё равно никто не достучится, поднимать заранее нет смысла
      if (!webServerRunning) {
        server.begin();
        webServerRunning = true;
      }
    } else if (now - wifiStateChangeMs >= WIFI_RETRY_INTERVAL_MS) {
      // не подключились за отведённое время - пробуем заново
      WiFi.disconnect();
      connectToWifi();
      wifiStateChangeMs = now;
    }
  } else { // WIFI_ST_CONNECTED
    if (WiFi.status() != WL_CONNECTED) {
      // связь пропала - возвращаемся в режим переподключения и
      // останавливаем веб-сервер (незачем ему висеть без сети)
      wifiState = WIFI_ST_CONNECTING;
      wifiStateChangeMs = now;
      connectToWifi();

      if (webServerRunning) {
        server.stop();
        webServerRunning = false;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// НАСТРОЙКА WI-FI ЧЕРЕЗ ПОРТАЛ (WiFiManager) - режим MODE_WIFI_SETUP_ACTIVE
// ---------------------------------------------------------------------------
// Вход в этот режим полностью ручной (через экран подтверждения, см. ниже) -
// ESP32 поднимает собственную точку доступа "Budka-Setup", пользователь
// подключается к ней с телефона/компьютера, открывает браузер (обычно
// captive-portal сам предложит открыть страницу настройки), выбирает сеть
// из списка и вводит пароль через обычную веб-форму - НЕ через энкодер.
//
// Работает в НЕБЛОКИРУЮЩЕМ режиме (setConfigPortalBlocking(false)) - весь
// остальной код (датчики, термостат, SSR/ЭМР, безопасность) продолжает
// работать без пауз, портал обслуживается через wifiManager.process()
// в loop(), как и наш собственный веб-сервер через server.handleClient().
WiFiManager wifiManager;

const unsigned long WIFI_SETUP_PORTAL_TIMEOUT_S = 300; // 5 минут - автоотмена, если никто не настроил
bool wifiSetupCredentialsSaved = false; // выставляется колбэком при успешном сохранении новых данных

// Колбэк WiFiManager - вызывается, когда пользователь успешно ввёл и
// сохранил новые учётные данные через веб-форму портала.
void onWifiManagerSaveConfig() {
  wifiSetupCredentialsSaved = true;
}

// Запускает портал настройки - вызывается один раз при подтверждённом входе
// в MODE_WIFI_SETUP_ACTIVE.
void startWifiSetupPortal() {
  // наш собственный веб-сервер (главная страница термостата) на время
  // настройки останавливаем - порт 80 в этот момент занят порталом
  // WiFiManager, и в любом случае устройство сейчас не в домашней сети
  // как обычно, а поднимает свою отдельную точку доступа
  if (webServerRunning) {
    server.stop();
    webServerRunning = false;
  }

  wifiSetupCredentialsSaved = false;
  wifiManager.setSaveConfigCallback(onWifiManagerSaveConfig);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(WIFI_SETUP_PORTAL_TIMEOUT_S);
  wifiManager.startConfigPortal("Budka-Setup");
}

// Вызывается каждую итерацию loop(), пока активен MODE_WIFI_SETUP_ACTIVE.
// Обслуживает сам портал и решает, когда пора выходить из этого режима.
void updateWifiSetupPortal() {
  wifiManager.process();

  if (wifiSetupCredentialsSaved) {
    // пользователь успешно ввёл и сохранил новую сеть - WiFiManager сам
    // передаёт эти данные ESP32 (WiFi.begin(ssid,pass) внутри), и чип сам
    // помнит их в своей внутренней NVS для последующих загрузок - никакой
    // дополнительной пометки с нашей стороны не требуется
    exitWifiSetupMode();
    return;
  }

  if (!wifiManager.getConfigPortalActive()) {
    // портал сам завершился (истёк WIFI_SETUP_PORTAL_TIMEOUT_S без
    // успешной настройки) - просто возвращаемся к обычной работе
    exitWifiSetupMode();
  }
}

// Общий выход из режима настройки - используется и при успехе, и при
// таймауте, и при ручной отмене длинным нажатием.
void exitWifiSetupMode() {
  if (wifiManager.getConfigPortalActive()) {
    wifiManager.stopConfigPortal();
  }

  systemMode = MODE_NORMAL;
  lastEncoderActivityMs = millis(); // не должно тут же снова сработать автовыключение подсветки

  // запускаем переподключение к Wi-Fi (по новым данным, если только что
  // настроили, либо по старым) - веб-сервер сам поднимется чуть позже,
  // как только updateWifiConnection() зафиксирует реальное подключение
  // (см. её логику) - здесь server.begin() специально НЕ вызывается
  startWifiConnection();
}

// ---------------------------------------------------------------------------
// OTA - АВТООБНОВЛЕНИЕ ПРОШИВКИ С GITHUB
// ---------------------------------------------------------------------------
// Версия и firmware.bin хранятся ПРЯМО В РЕПОЗИТОРИИ (не как GitHub Release) -
// оба файла отдаются через raw.githubusercontent.com одним и тем же доменом,
// что упрощает работу с TLS (один хост вместо нескольких доменов с разными
// сертификатами у GitHub Releases/API). Разместите в репозитории:
//   /firmware/version.json  - {"version": "1.1.0"}
//   /firmware/firmware.bin  - скомпилированный .bin новой версии
//
const char* OTA_VERSION_URL  = "https://raw.githubusercontent.com/jekavovikovich/budka/main/firmware/version.json";
const char* OTA_FIRMWARE_URL = "https://raw.githubusercontent.com/jekavovikovich/budka/main/firmware/firmware.bin";

// ---------------------------------------------------------------------------
// ДИАГНОСТИКА ПЕРЕЗАГРУЗОК И ЗАВИСАНИЙ
// ---------------------------------------------------------------------------
// Цель - после очередного зависания/сброса понять, ПОЧЕМУ он случился.
// Что запоминается:
//  - причина последнего сброса (esp_reset_reason()) и счётчики по типам;
//  - сколько проработал предыдущий запуск и когда была его ПОСЛЕДНЯЯ
//    активность (по NTP-времени) - т.е. когда примерно всё остановилось;
//  - этап (место в loop()/setup()), на котором контроллер был в последний момент;
//  - состояние нагрева (SSR/ЭМР), аварийные флаги и температура воздуха
//    в этот момент;
//  - минимум свободной памяти (признак утечки), самая долгая итерация loop(),
//    разрывы Wi-Fi и их причины.
// Хранится в двух местах:
//  - RTC-память (RTC_NOINIT_ATTR): переживает любые сбросы, КРОМЕ полного
//    пропадания питания; обновляется раз в 10 с - точная картина;
//  - NVS (флеш): переживает и отключение питания; пишется редко (раз в 10
//    минут + пара записей за запуск), чтобы не изнашивать флеш. Если питание
//    пропадало или контроллер перезапускали вручную, "время последней
//    активности" берётся отсюда с точностью до 10 минут.
// версия прошивки определена ниже (рядом с OTA), а нужна уже здесь - для баннера в UART
extern const char* FIRMWARE_VERSION;

const uint32_t DIAG_RTC_MAGIC = 0xB0D0CA7Eu;
const unsigned long DIAG_RTC_INTERVAL_MS = 10000UL;
const unsigned long DIAG_NVS_INTERVAL_MS = 10UL * 60UL * 1000UL;
// Вывод в UART (USB-порт, 115200 бод): баннер при запуске с причиной сброса и
// данными прошлого запуска, строка состояния раз в DIAG_UART_STATUS_MS и каждое
// событие сразу. Отключается заменой 1 на 0 - тогда код вывода не компилируется.
// Тексты в UART - латиницей, чтобы не зависеть от кодировки монитора порта.
#define DIAG_UART_ENABLED 1
const unsigned long DIAG_UART_STATUS_MS = 30000UL;
#if DIAG_UART_ENABLED
#define DLOG(...) Serial.printf(__VA_ARGS__)
#else
#define DLOG(...) do {} while (0)
#endif

const uint8_t DIAG_LOG_SIZE = 8;       // сколько последних перезагрузок помним
const uint8_t DIAG_REASON_SLOTS = 16;  // коды esp_reset_reason() 0..15

// этапы работы - номера используются страницей графиков (STAGE_NAMES)
enum DiagStage : uint8_t {
  STAGE_LOOP = 0, STAGE_BUTTONS = 1, STAGE_WIFI = 2, STAGE_OTA_CHECK = 3,
  STAGE_WEB = 4, STAGE_OTA_DOWNLOAD = 5, STAGE_SENSORS = 6, STAGE_CURRENT = 7,
  STAGE_THERMOSTAT = 8, STAGE_DISPLAY = 9, STAGE_SETUP = 10
};

// --- лента событий (RAM, последние 32): что и когда происходило -
// аварии, Wi-Fi, датчики, задержки цикла и т.п. Нужна, чтобы увидеть
// последовательность событий ПЕРЕД проблемой; после перезагрузки обнуляется
// (для постоянного следа есть журнал перезагрузок ниже и вывод в UART).
enum DiagEventType : uint8_t {
  EV_BOOT = 1, EV_ALARM_SET = 2, EV_ALARM_CLR = 3, EV_WIFI_DOWN = 4, EV_WIFI_UP = 5,
  EV_NTP = 6, EV_OTA_CHECK = 7, EV_SENSOR_BAD = 8, EV_SENSOR_REINIT = 9, EV_STALL = 10,
  EV_HEAP = 11, EV_EMERGENCY = 12, EV_EMR = 13, EV_ENC_NOISE = 14
};
const char* const DIAG_EVENT_NAMES[15] = {
  "?", "BOOT", "ALARM_SET", "ALARM_CLEAR", "WIFI_DOWN", "WIFI_UP", "NTP_SYNC", "OTA_CHECK",
  "SENSOR_BAD", "SENSOR_REINIT", "LOOP_STALL", "HEAP_DROP", "EMERGENCY_MODE", "EMR", "ENC_NOISE"
};
struct DiagEvent {             // 20 байт
  uint32_t uptimeSec;
  uint32_t epoch;              // 0 - NTP ещё не было
  int32_t  a;
  int32_t  b;
  uint8_t  type;
  uint8_t  pad[3];
};
const uint8_t DIAG_EVENT_RING = 32;
DiagEvent diagEvents[DIAG_EVENT_RING];
uint8_t diagEventHead = 0;     // куда писать следующее событие
uint8_t diagEventCount = 0;
unsigned long diagEvLastMs[16];
int32_t diagEvLastA[16];
uint32_t diagEventSuppressed = 0; // сколько повторов подавлено (одинаковое событие чаще раза в 30 с)

// счётчики датчиков температуры (0 - коврик, 1 - воздух, 2 - улица)
uint32_t diagSensorDisc[3];    // чтение "нет связи" (-127)
uint32_t diagSensorPor[3];     // чтение 85.0 °C = датчик перезагрузился по питанию
uint32_t diagSensorReinits[3]; // сколько раз пытались переинициализировать

uint32_t diagSsrSwitches = 0;
uint32_t diagEmrSwitches = 0;
uint32_t diagLoopsPerSec = 0;
uint32_t diagStackMinFree = 0; // минимум свободного стека loop(), байт
unsigned long diagWifiUpMs = 0;

struct DiagSnapshot {          // 20 байт: что запоминаем о работающем контроллере
  uint32_t uptimeSec;
  uint32_t epoch;              // unix-время, 0 - NTP ещё не было
  uint32_t minHeap;            // минимум свободной памяти с запуска, байт
  uint32_t maxLoopMs;          // самая долгая итерация loop() с запуска, мс
  int16_t  airTenths;          // температура воздуха, десятые доли градуса (-1270 - нет данных)
  uint8_t  flags;              // bit0 - SSR включён, bit1 - ЭМР замкнут
  uint8_t  alarms;             // alarmFlags
};

struct DiagRtcBlock {
  uint32_t magic;
  DiagSnapshot snap;
  uint8_t stage;               // не входит в контрольную сумму - меняется очень часто
  uint8_t pad[3];
  uint32_t check;
};
RTC_NOINIT_ATTR DiagRtcBlock diagRtc;

struct DiagBootRecord {        // 24 байта: чем закончился предыдущий запуск
  DiagSnapshot last;
  uint8_t reason;              // причина сброса, с которого начался СЛЕДУЮЩИЙ запуск
  uint8_t stage;
  uint8_t source;              // 0 - данных нет, 1 - из RTC (точно), 2 - из NVS (+-10 мин)
  uint8_t pad;
};

DiagBootRecord diagLog[DIAG_LOG_SIZE];   // [0] - самая свежая
uint8_t diagLogCount = 0;
uint16_t diagReasonCount[DIAG_REASON_SLOTS];
uint8_t diagResetReason = 0;
uint32_t diagMaxLoopMs = 0;
uint32_t diagWifiConnects = 0;
uint32_t diagWifiDisconnects = 0;
uint8_t diagWifiLastReason = 0;
unsigned long diagLastRtcMs = 0;
unsigned long diagLastNvsMs = 0;
// фоновая проверка версии OTA (см. otaCheckTask): сколько раз выполнялась,
// сколько длилась последняя, чем закончилась (0 - ещё не было, 1 - успешно, 2 - неудача)
volatile uint32_t diagOtaCheckCount = 0;
volatile uint32_t diagOtaCheckLastMs = 0;
volatile uint8_t diagOtaCheckResult = 0;

// Сколько МАКСИМУМ времени (мс) контроллер провёл в каждом этапе за один заход -
// показывает, какой именно участок loop() блокирует выполнение дольше всего.
const uint8_t DIAG_STAGE_COUNT = 11;
uint32_t diagStageMaxMs[DIAG_STAGE_COUNT];
uint8_t diagStageCur = STAGE_SETUP;
unsigned long diagStageEnterMs = 0;

void diagStage(uint8_t st) {
  unsigned long now = millis();
  if (diagStageCur < DIAG_STAGE_COUNT) {
    uint32_t dt = (uint32_t)(now - diagStageEnterMs);
    if (dt > diagStageMaxMs[diagStageCur]) diagStageMaxMs[diagStageCur] = dt;
    if (dt >= 500 && diagStageCur != STAGE_SETUP) diagEvent(EV_STALL, diagStageCur, (int32_t)dt);
  }
  diagStageCur = st;
  diagStageEnterMs = now;
  diagRtc.stage = st;
}

uint32_t diagUptimeSec() { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

uint32_t diagRtcChecksum() {
  const uint8_t* b = (const uint8_t*)&diagRtc.snap;
  uint32_t h = DIAG_RTC_MAGIC;
  for (size_t i = 0; i < sizeof(DiagSnapshot); i++) h = h * 31u + b[i];
  return h;
}

// Без параметров намеренно: Arduino генерирует прототипы функций ДО определений
// структур, поэтому функции с параметрами пользовательских типов здесь не
// используются. Заполняет diagRtc.snap.
void diagFillSnapshot() {
  DiagSnapshot &sn = diagRtc.snap;
  sn.uptimeSec = diagUptimeSec();
  sn.epoch     = timeSynced ? (uint32_t)time(nullptr) : 0;
  sn.minHeap   = ESP.getMinFreeHeap();
  sn.maxLoopMs = diagMaxLoopMs;
  sn.airTenths = tempAir;
  sn.flags     = (ssrState ? 1 : 0) | (emrClosed ? 2 : 0);
  sn.alarms    = alarmFlags;
}

// Флаг "сейчас выполняется долгая блокирующая операция" в NVS: если после
// неё контроллер так и не вернулся (зависание при выключенном watchdog и
// последующее отключение питания), при следующем запуске по флагу видно, где
// именно он завис. Пишется только вокруг редких операций.
void diagBusySet(uint8_t st) {
  diagStage(st);
  Preferences pr;
  if (!pr.begin("diag", false)) return;
  pr.putUChar("busy", st);
  pr.end();
}
void diagBusyClear() {
  diagStage(STAGE_LOOP);
  Preferences pr;
  if (!pr.begin("diag", false)) return;
  pr.putUChar("busy", 0);
  pr.end();
}
struct DiagBusy {
  explicit DiagBusy(uint8_t st) { diagBusySet(st); }
  ~DiagBusy() { diagBusyClear(); }
};

const char* diagReasonNameEn(uint8_t r) {
  switch (r) {
    case 1: return "POWERON";   case 2: return "EXT_RESET"; case 3: return "SOFTWARE";
    case 4: return "PANIC";     case 5: return "INT_WDT";   case 6: return "TASK_WDT";
    case 7: return "OTHER_WDT"; case 8: return "DEEPSLEEP"; case 9: return "BROWNOUT";
    case 10: return "SDIO";     case 11: return "USB";      case 12: return "JTAG";
    case 13: return "EFUSE";    case 14: return "PWR_GLITCH"; case 15: return "CPU_LOCKUP";
    default: return "UNKNOWN";
  }
}

const char* diagStageNameEn(uint8_t st) {
  switch (st) {
    case STAGE_LOOP: return "loop";           case STAGE_BUTTONS: return "buttons";
    case STAGE_WIFI: return "wifi/time";      case STAGE_OTA_CHECK: return "ota-check";
    case STAGE_WEB: return "web";             case STAGE_OTA_DOWNLOAD: return "ota-download";
    case STAGE_SENSORS: return "sensors";     case STAGE_CURRENT: return "current";
    case STAGE_THERMOSTAT: return "thermostat"; case STAGE_DISPLAY: return "display";
    case STAGE_SETUP: return "setup";         default: return "?";
  }
}

// Unix-время -> "YYYY-MM-DD hh:mm:ss UTC" (в UART время пишем в UTC, без часовых поясов)
void diagFmtUtc(char* out, size_t n, uint32_t epoch) {
  if (epoch == 0) { snprintf(out, n, "unknown"); return; }
  time_t t = (time_t)epoch;
  struct tm tmv;
  gmtime_r(&t, &tmv);
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d UTC", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

// Добавляет событие в ленту и печатает его в UART. Одинаковое событие (тот же
// тип и параметр a) чаще раза в 30 с подавляется, чтобы лента и UART не
// забивались повторами при "дребезге" (счётчики при этом продолжают расти).
// Вызывается ТОЛЬКО из потока loop().
void diagEvent(uint8_t type, int32_t a, int32_t b) {
  unsigned long now = millis();
  if (type < 16) {
    if (diagEvLastMs[type] != 0 && diagEvLastA[type] == a && (now - diagEvLastMs[type]) < 30000UL) {
      diagEventSuppressed++;
      return;
    }
    diagEvLastMs[type] = now;
    diagEvLastA[type] = a;
  }
  DiagEvent &e = diagEvents[diagEventHead];
  e.uptimeSec = diagUptimeSec();
  e.epoch = timeSynced ? (uint32_t)time(nullptr) : 0;
  e.a = a;
  e.b = b;
  e.type = type;
  diagEventHead = (diagEventHead + 1) % DIAG_EVENT_RING;
  if (diagEventCount < DIAG_EVENT_RING) diagEventCount++;
  DLOG("[%lus] EVENT %s a=%ld b=%ld\n", (unsigned long)e.uptimeSec,
       DIAG_EVENT_NAMES[type < 15 ? type : 0], (long)a, (long)b);
}

// Хуки из кода датчиков: плохое чтение и переинициализация.
void diagSensorBad(uint8_t idx, int16_t raw) {
  if (idx > 2) return;
  if (raw == DS18B20_POWER_ON_RESET_RAW) {
    diagSensorPor[idx]++;
    diagEvent(EV_SENSOR_BAD, idx, 85);
  } else {
    diagSensorDisc[idx]++;
    diagEvent(EV_SENSOR_BAD, idx, -127);
  }
}
void diagSensorReinit(uint8_t idx) {
  if (idx > 2) return;
  diagSensorReinits[idx]++;
  diagEvent(EV_SENSOR_REINIT, idx, (int32_t)diagSensorReinits[idx]);
}

int diagOtaImageState() {
  esp_ota_img_states_t st;
  const esp_partition_t* run = esp_ota_get_running_partition();
  if (run != NULL && esp_ota_get_state_partition(run, &st) == ESP_OK) return (int)st;
  return -1;
}

// Вызывается ОДИН раз, в самом начале setup() (после установки безопасных
// уровней реле): разбирает, чем закончился предыдущий запуск, дописывает это
// в журнал и заново заводит RTC-блок.
void diagPrintBootReport();

void diagBoot() {
#if DIAG_UART_ENABLED
  Serial.begin(115200);
#endif
  esp_reset_reason_t rr = esp_reset_reason();
  diagResetReason = (uint8_t)rr;

  Preferences pr;
  bool nvsOk = pr.begin("diag", false);

  DiagBootRecord rec;
  memset(&rec, 0, sizeof(rec));
  rec.reason = (uint8_t)rr;

  bool rtcValid = (diagRtc.magic == DIAG_RTC_MAGIC) && (diagRtc.check == diagRtcChecksum());
  if (rtcValid) {
    rec.last = diagRtc.snap;
    rec.stage = diagRtc.stage;
    rec.source = 1;
  } else if (nvsOk && pr.getBytesLength("hb") == sizeof(DiagSnapshot)) {
    pr.getBytes("hb", &rec.last, sizeof(DiagSnapshot));
    rec.source = 2;
  }
  if (nvsOk) {
    uint8_t busy = pr.getUChar("busy", 0);
    if (busy != 0) rec.stage = busy; // прошлый запуск не вышел из долгой операции
  }

  diagLogCount = 0;
  if (nvsOk) {
    size_t n = pr.getBytesLength("log");
    if (n > 0 && (n % sizeof(DiagBootRecord)) == 0 && n <= sizeof(diagLog)) {
      pr.getBytes("log", diagLog, n);
      diagLogCount = (uint8_t)(n / sizeof(DiagBootRecord));
    }
    if (pr.getBytesLength("cnt") == sizeof(diagReasonCount)) {
      pr.getBytes("cnt", diagReasonCount, sizeof(diagReasonCount));
    }
  }
  if (diagLogCount > DIAG_LOG_SIZE - 1) diagLogCount = DIAG_LOG_SIZE - 1;
  for (int i = diagLogCount; i > 0; i--) diagLog[i] = diagLog[i - 1];
  diagLog[0] = rec;
  diagLogCount++;
  if (rr < DIAG_REASON_SLOTS && diagReasonCount[rr] < 65535) diagReasonCount[rr]++;

  if (nvsOk) {
    pr.putBytes("log", diagLog, diagLogCount * sizeof(DiagBootRecord));
    pr.putBytes("cnt", diagReasonCount, sizeof(diagReasonCount));
    pr.putUChar("busy", STAGE_SETUP); // снимается в конце setup(): если не снялся - запуск завис/упал при старте
    pr.end();
  }

  memset(&diagRtc, 0, sizeof(diagRtc));
  diagRtc.magic = DIAG_RTC_MAGIC;
  diagFillSnapshot();
  diagRtc.stage = STAGE_SETUP;
  diagRtc.check = diagRtcChecksum();
  diagLastRtcMs = millis();
  diagLastNvsMs = millis();
  diagStageCur = STAGE_SETUP;
  diagStageEnterMs = millis();

  diagPrintBootReport();
  diagEvent(EV_BOOT, (int32_t)rr, 0);
}

// Баннер в UART: версия, причина сброса, чем закончился предыдущий запуск,
// счётчики сбросов и журнал.
void diagPrintBootReport() {
#if DIAG_UART_ENABLED
  DLOG("\n==================================================\n");
  DLOG("Kennel Thermostat v%s (built %s %s)\n", FIRMWARE_VERSION, __DATE__, __TIME__);
  DLOG("Reset reason: %s (%u)\n", diagReasonNameEn(diagResetReason), (unsigned)diagResetReason);
  DLOG("Chip: %s rev %u, %u MHz, flash %u MB, sketch %u KB (free %u KB)\n", ESP.getChipModel(),
       (unsigned)ESP.getChipRevision(), (unsigned)ESP.getCpuFreqMHz(), (unsigned)(ESP.getFlashChipSize() / 1048576),
       (unsigned)(ESP.getSketchSize() / 1024), (unsigned)(ESP.getFreeSketchSpace() / 1024));
  DLOG("SDK: %s\n", ESP.getSdkVersion());
  DLOG("Heap: free %u, min %u, max block %u\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
       (unsigned)ESP.getMaxAllocHeap());
  const DiagBootRecord &r = diagLog[0];
  if (r.source == 0) {
    DLOG("Previous run: no data\n");
  } else {
    char tbuf[40];
    diagFmtUtc(tbuf, sizeof(tbuf), r.last.epoch);
    DLOG("Previous run (%s): lasted %lu s, last activity %s, stage '%s'\n",
         r.source == 1 ? "RTC, exact" : "NVS, +-10min", (unsigned long)r.last.uptimeSec, tbuf, diagStageNameEn(r.stage));
    DLOG("  SSR=%d EMR=%d air=%d (x0.1C) alarms=0x%02X min_heap=%lu max_loop=%lums\n",
         (r.last.flags & 1) ? 1 : 0, (r.last.flags & 2) ? 1 : 0, (int)r.last.airTenths,
         (unsigned)r.last.alarms, (unsigned long)r.last.minHeap, (unsigned long)r.last.maxLoopMs);
  }
  DLOG("Reset counters:");
  for (uint8_t i = 0; i < DIAG_REASON_SLOTS; i++) {
    if (diagReasonCount[i]) DLOG(" %s=%u", diagReasonNameEn(i), (unsigned)diagReasonCount[i]);
  }
  DLOG("\nLog (newest first):\n");
  for (uint8_t i = 0; i < diagLogCount; i++) {
    const DiagBootRecord &q = diagLog[i];
    DLOG("  #%u reason=%s prev_up=%lus stage=%s src=%u\n", (unsigned)(i + 1), diagReasonNameEn(q.reason),
         (unsigned long)q.last.uptimeSec, diagStageNameEn(q.stage), (unsigned)q.source);
  }
  DLOG("==================================================\n");
#endif
}

// Строка состояния в UART (раз в DIAG_UART_STATUS_MS).
void diagPrintStatus() {
#if DIAG_UART_ENABLED
  DLOG("[%lus] T out=%d air=%d mat=%d set=%dC SSR=%d EMR=%d I=%dmA alarm=0x%02X emerg=%d | heap=%u/%u/%u wifi=%s rssi=%d disc=%lu | lps=%lu maxloop=%lums stack_free=%lu | encinv=%lu/%lu\n",
       (unsigned long)diagUptimeSec(), (int)tempOut, (int)tempAir, (int)tempMat, (int)setpointC,
       ssrState ? 1 : 0, emrClosed ? 1 : 0, (int)currentAmpsHundredths * 10, (unsigned)alarmFlags,
       emergencyModeActive ? 1 : 0,
       (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getMaxAllocHeap(),
       wifiState == WIFI_ST_CONNECTED ? "up" : "down", wifiState == WIFI_ST_CONNECTED ? (int)WiFi.RSSI() : 0,
       (unsigned long)diagWifiDisconnects, (unsigned long)diagLoopsPerSec, (unsigned long)diagMaxLoopMs,
       (unsigned long)diagStackMinFree, (unsigned long)encoderInvalidCount, (unsigned long)encoderIsrCount);
#endif
}

// Вызывается каждую итерацию loop(): считает самую долгую итерацию, следит за
// изменениями состояния (аварии, SSR/ЭМР, Wi-Fi, время) и пишет их в ленту событий,
// раз в 10 с обновляет RTC-блок и медленные показатели, раз в 10 минут - NVS,
// раз в DIAG_UART_STATUS_MS - строку состояния в UART.
void diagTick() {
  static unsigned long prevLoopMs = 0;
  static unsigned long prevSlowMs = 0;
  static unsigned long lastStatusMs = 0;
  static uint32_t loopCount = 0, prevLoopCount = 0;
  static uint32_t prevEncInvalid = 0, heapMinReported = 0;
  static uint8_t prevAlarm = 0;
  static bool prevSsr = false, prevEmr = false, prevEmerg = false, prevSync = false;
  static uint32_t prevDisc = 0, prevConn = 0;
  unsigned long now = millis();

  loopCount++;
  if (prevLoopMs != 0) {
    unsigned long dt = now - prevLoopMs;
    if (dt > diagMaxLoopMs) diagMaxLoopMs = dt;
  }
  prevLoopMs = now;
  if (prevSlowMs == 0) { prevSlowMs = now; lastStatusMs = now; }

  // --- наблюдатели изменений (дёшево, каждую итерацию) ---
  uint8_t al = alarmFlags;
  if (al != prevAlarm) {
    uint8_t setBits = al & ~prevAlarm;
    uint8_t clrBits = prevAlarm & ~al;
    if (setBits) diagEvent(EV_ALARM_SET, setBits, al);
    if (clrBits) diagEvent(EV_ALARM_CLR, clrBits, al);
    prevAlarm = al;
  }
  if (ssrState != prevSsr) {
    prevSsr = ssrState;
    diagSsrSwitches++;
    DLOG("[%lus] SSR %s (air=%d set=%dC)\n", (unsigned long)diagUptimeSec(), ssrState ? "ON" : "OFF", (int)tempAir, (int)setpointC);
  }
  if (emrClosed != prevEmr) {
    prevEmr = emrClosed;
    diagEmrSwitches++;
    diagEvent(EV_EMR, emrClosed ? 1 : 0, 0);
  }
  if (emergencyModeActive != prevEmerg) {
    prevEmerg = emergencyModeActive;
    diagEvent(EV_EMERGENCY, emergencyModeActive ? 1 : 0, 0);
  }
  if (timeSynced && !prevSync) {
    prevSync = true;
    diagEvent(EV_NTP, 0, 0);
  }
  if (diagWifiDisconnects != prevDisc) {
    prevDisc = diagWifiDisconnects;
    diagEvent(EV_WIFI_DOWN, diagWifiLastReason, diagWifiUpMs ? (int32_t)((now - diagWifiUpMs) / 1000) : 0);
    diagWifiUpMs = 0;
  }
  if (diagWifiConnects != prevConn) {
    prevConn = diagWifiConnects;
    diagWifiUpMs = now;
    diagEvent(EV_WIFI_UP, (int32_t)WiFi.RSSI(), (int32_t)diagWifiConnects);
  }

  // --- раз в 10 с: RTC-блок и медленные показатели ---
  if (now - diagLastRtcMs >= DIAG_RTC_INTERVAL_MS) {
    diagLastRtcMs = now;
    diagFillSnapshot();
    diagRtc.check = diagRtcChecksum();

    unsigned long span = now - prevSlowMs;
    if (span > 0) diagLoopsPerSec = (uint32_t)((uint64_t)(loopCount - prevLoopCount) * 1000ULL / span);
    prevLoopCount = loopCount;
    prevSlowMs = now;

    uint32_t freeStack = uxTaskGetStackHighWaterMark(NULL);
    if (diagStackMinFree == 0 || freeStack < diagStackMinFree) diagStackMinFree = freeStack;

    uint32_t mh = ESP.getMinFreeHeap();
    if (heapMinReported == 0) heapMinReported = mh;
    if (mh + 8192 <= heapMinReported) { // новый минимум свободной памяти ниже прежнего на 8 КБ и более
      heapMinReported = mh;
      diagEvent(EV_HEAP, (int32_t)mh, (int32_t)ESP.getFreeHeap());
    }

    uint32_t inv = encoderInvalidCount;
    if (inv - prevEncInvalid >= 5) diagEvent(EV_ENC_NOISE, (int32_t)(inv - prevEncInvalid), (int32_t)inv);
    prevEncInvalid = inv;
  }

  if (now - diagLastNvsMs >= DIAG_NVS_INTERVAL_MS) {
    diagLastNvsMs = now;
    Preferences pr;
    if (pr.begin("diag", false)) {
      pr.putBytes("hb", &diagRtc.snap, sizeof(DiagSnapshot));
      pr.end();
    }
  }

  if (now - lastStatusMs >= DIAG_UART_STATUS_MS) {
    lastStatusMs = now;
    diagPrintStatus();
  }
}

// События Wi-Fi: считаем подключения/разрывы и запоминаем причину последнего
// разрыва (только счётчики - обработчик выполняется в потоке событий Wi-Fi).
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2)
void diagOnWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    diagWifiDisconnects++;
    diagWifiLastReason = info.wifi_sta_disconnected.reason;
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    diagWifiConnects++;
  }
}
#endif

// Строка для вставки в JSON: убираем кавычки и обратные слэши (на всякий случай)
String diagJsonSafe(const char* in) {
  String out;
  if (in == NULL) return out;
  for (const char* c = in; *c; c++) out += (*c == '"' || *c == '\\' || (unsigned char)*c < 32) ? ' ' : *c;
  return out;
}

// JSON для страницы диагностики (/diag): состояние, счётчики, журнал и лента событий.
void handleApiDiag() {
  String js;
  js.reserve(4200);
  js += F("{\"up\":");   js += diagUptimeSec();
  js += F(",\"now\":");  js += (timeSynced ? (uint32_t)time(nullptr) : 0u);
  js += F(",\"rr\":");   js += diagResetReason;
  js += F(",\"heap\":[");
  js += ESP.getFreeHeap(); js += ','; js += ESP.getMinFreeHeap(); js += ','; js += ESP.getMaxAllocHeap();
  js += F("],\"rssi\":"); js += (wifiState == WIFI_ST_CONNECTED ? (int)WiFi.RSSI() : 0);
  js += F(",\"wc\":");   js += diagWifiConnects;
  js += F(",\"wd\":");   js += diagWifiDisconnects;
  js += F(",\"wr\":");   js += diagWifiLastReason;
  js += F(",\"ml\":");   js += diagMaxLoopMs;
  js += F(",\"sm\":[");
  for (uint8_t i = 0; i < DIAG_STAGE_COUNT; i++) {
    if (i) js += ',';
    js += diagStageMaxMs[i];
  }
  js += F("],\"oc\":[");
  js += diagOtaCheckCount; js += ','; js += diagOtaCheckLastMs; js += ','; js += diagOtaCheckResult;
  js += F("],\"cnt\":[");
  for (uint8_t i = 0; i < DIAG_REASON_SLOTS; i++) {
    if (i) js += ',';
    js += diagReasonCount[i];
  }
  js += F("]");
  js += F(",\"st\":[");
  js += tempOut; js += ','; js += tempAir; js += ','; js += tempMat; js += ',';
  js += (int)setpointC; js += ','; js += ((int)currentAmpsHundredths * 10); js += ',';
  js += (ssrState ? 1 : 0); js += ','; js += (emrClosed ? 1 : 0); js += ',';
  js += alarmFlags; js += ','; js += (emergencyModeActive ? 1 : 0);
  js += F("],\"sn\":[");
  {
    const bool fnd[3] = { matFound, airFound, outFound };
    const uint16_t streak[3] = { matErrorStreak, airErrorStreak, outErrorStreak };
    const uint8_t att[3] = { matReinitAttempts, airReinitAttempts, outReinitAttempts };
    for (uint8_t i = 0; i < 3; i++) {
      if (i) js += ',';
      js += '['; js += (fnd[i] ? 1 : 0); js += ','; js += streak[i]; js += ','; js += att[i]; js += ',';
      js += diagSensorDisc[i]; js += ','; js += diagSensorPor[i]; js += ','; js += diagSensorReinits[i]; js += ']';
    }
  }
  js += F("],\"en\":["); js += encoderIsrCount; js += ','; js += encoderInvalidCount;
  js += F("],\"cs\":["); js += (int)currentNoiseBaselineRms; js += ','; js += (currentBaselineCalibrated ? 1 : 0);
  js += F("],\"sy\":[");
  js += (uint32_t)ESP.getCpuFreqMHz(); js += ','; js += (uint32_t)ESP.getChipRevision(); js += ',';
  js += (uint32_t)(ESP.getFlashChipSize() / 1048576); js += ','; js += (uint32_t)(ESP.getSketchSize() / 1024); js += ',';
  js += (uint32_t)(ESP.getFreeSketchSpace() / 1024); js += ','; js += diagStackMinFree; js += ',';
  js += diagLoopsPerSec; js += ','; js += diagSsrSwitches; js += ','; js += diagEmrSwitches; js += ',';
  js += diagOtaImageState(); js += ',';
  js += (wifiState == WIFI_ST_CONNECTED ? (int)WiFi.channel() : 0); js += ',';
  js += (uint32_t)((wifiState == WIFI_ST_CONNECTED && diagWifiUpMs) ? (millis() - diagWifiUpMs) / 1000 : 0); js += ',';
  js += diagEventSuppressed;
  js += F("],\"ip\":\"");
  if (wifiState == WIFI_ST_CONNECTED) js += WiFi.localIP().toString();
  js += F("\",\"chip\":\""); js += diagJsonSafe(ESP.getChipModel());
  js += F("\",\"sdk\":\""); js += diagJsonSafe(ESP.getSdkVersion());
  js += F("\"");
  js += F(",\"log\":[");
  for (uint8_t i = 0; i < diagLogCount; i++) {
    const DiagBootRecord &r = diagLog[i];
    char buf[120];
    snprintf(buf, sizeof(buf), "%s[%u,%u,%u,%lu,%lu,%lu,%lu,%d,%u,%u]",
             i ? "," : "",
             (unsigned)r.reason, (unsigned)r.stage, (unsigned)r.source,
             (unsigned long)r.last.epoch, (unsigned long)r.last.uptimeSec,
             (unsigned long)r.last.minHeap, (unsigned long)r.last.maxLoopMs,
             (int)r.last.airTenths, (unsigned)r.last.flags, (unsigned)r.last.alarms);
    js += buf;
  }
  js += F("],\"ev\":[");
  for (uint8_t i = 0; i < diagEventCount; i++) {
    uint8_t idx = (uint8_t)((diagEventHead + DIAG_EVENT_RING - 1 - i) % DIAG_EVENT_RING); // от свежих к старым
    const DiagEvent &e = diagEvents[idx];
    char buf[96];
    snprintf(buf, sizeof(buf), "%s[%lu,%lu,%u,%ld,%ld]", i ? "," : "",
             (unsigned long)e.uptimeSec, (unsigned long)e.epoch, (unsigned)e.type, (long)e.a, (long)e.b);
    js += buf;
  }
  js += F("]}");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", js);
}

// Версия ЭТОЙ прошивки - обновляйте вручную перед каждой новой компиляцией
// и публикацией в репозиторий (простой формат X.Y.Z, сравнение ниже это
// подразумевает).
const char* FIRMWARE_VERSION = "1.1.7";

// Стартовый экран с версией прошивки - переменные состояния объявлены
// раньше по файлу (см. рядом с lastRenderedMode), т.к. используются в
// updateDisplay(), которая находится в файле выше этого места.
void drawVersionSplashScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 120);
  tft.print("Kennel Thermostat");

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 150);
  tft.print("Firmware v");
  tft.print(FIRMWARE_VERSION);
}

const unsigned long OTA_CHECK_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL; // раз в сутки

bool otaUpdateAvailable = false;
String otaAvailableVersion = "";
bool otaUpdateInProgress = false;
bool otaConfirmedTriggered = false; // выставляется из веб-обработчика, обрабатывается в loop()
unsigned long lastOtaCheckMs = 0;
bool otaCheckedOnce = false;

// Простое посемверное сравнение "X.Y.Z" - без внешних библиотек. Возвращает
// true, если remoteVersion СТРОГО новее currentVersion.
bool isVersionNewer(const String &remoteVersion, const String &currentVersion) {
  int remoteParts[3] = {0, 0, 0};
  int currentParts[3] = {0, 0, 0};
  sscanf(remoteVersion.c_str(), "%d.%d.%d", &remoteParts[0], &remoteParts[1], &remoteParts[2]);
  sscanf(currentVersion.c_str(), "%d.%d.%d", &currentParts[0], &currentParts[1], &currentParts[2]);

  for (int i = 0; i < 3; i++) {
    if (remoteParts[i] > currentParts[i]) return true;
    if (remoteParts[i] < currentParts[i]) return false;
  }
  return false; // версии равны
}

// Проверка версии прошивки выполняется В ОТДЕЛЬНОЙ ЗАДАЧЕ FreeRTOS, а не в
// loop(). Раньше она блокировала loop() на всё время DNS + TLS-рукопожатия +
// ответа (в диагностике - до 7,7 с), причём watchdog на это время отключался:
// нагрев, датчики, кнопки и веб-сервер в эти секунды не обслуживались, а при
// пропадании интернета в этот момент блокировка могла затянуться. Теперь
// сетевая часть живёт в своей задаче на ядре 0 (рядом со стеком Wi-Fi), а
// loop() только запускает её и забирает готовый результат.
//
// Данные между задачей и loop() передаются через простой буфер и флаги (никаких
// String и EEPROM из задачи): задача пишет буфер, затем поднимает флаг
// otaCheckResultReady; loop() читает буфер только после флага.
volatile bool otaCheckTaskRunning = false;
volatile bool otaCheckResultReady = false;
volatile bool otaCheckResultOk = false;
char otaCheckRemoteVersion[16] = "";

void otaCheckTask(void* arg) {
  unsigned long startMs = millis();
  bool ok = false;
  char ver[16];
  ver[0] = '\0';
  {
    WiFiClientSecure client;
    // ВАЖНО: проверка сертификата сервера ОТКЛЮЧЕНА (см. пояснение в начале
    // этого блока про хрупкость пиновки конкретного корневого сертификата
    // GitHub - он менялся несколько раз за последние годы). Риск: кто-то в
    // локальной сети между устройством и интернетом теоретически мог бы
    // подменить ответ. Для домашней сети это принятый компромисс, но при
    // желании можно заменить на client.setCACert(...) с актуальным
    // сертификатом raw.githubusercontent.com.
    client.setInsecure();
    client.setHandshakeTimeout(10); // TLS-рукопожатие - не дольше 10 с (по умолчанию 120 с)

    HTTPClient http;
    http.setConnectTimeout(4000); // не ждём связь с GitHub дольше, чем нужно
    http.setTimeout(4000);
    if (http.begin(client, OTA_VERSION_URL)) {
      int httpCode = http.GET();
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();

        // простой разбор JSON без библиотек - ищем "version":"X.Y.Z" вручную,
        // формат файла заведомо простой и полностью под нашим контролем
        int idx = payload.indexOf("\"version\"");
        if (idx >= 0) {
          int colon = payload.indexOf(':', idx);
          int q1 = payload.indexOf('"', colon + 1);
          int q2 = payload.indexOf('"', q1 + 1);
          if (colon >= 0 && q1 >= 0 && q2 > q1) {
            String remoteVersion = payload.substring(q1 + 1, q2);
            if (remoteVersion.length() > 0 && remoteVersion.length() < sizeof(ver)) {
              strcpy(ver, remoteVersion.c_str());
              ok = true;
            }
          }
        }
      }
      http.end();
    }
  } // клиент и HTTP-объект уничтожены ДО удаления задачи

  strcpy(otaCheckRemoteVersion, ver);
  otaCheckResultOk = ok;
  diagOtaCheckLastMs = (uint32_t)(millis() - startMs);
  diagOtaCheckResult = ok ? 1 : 2;
  diagOtaCheckCount++;
  __sync_synchronize();        // данные выше должны быть записаны ДО флага
  otaCheckResultReady = true;
  otaCheckTaskRunning = false;
  vTaskDelete(NULL);
}

// Запускает фоновую проверку (сама НЕ блокирует и возвращается сразу).
void checkForFirmwareUpdate() {
  if (wifiState != WIFI_ST_CONNECTED) return;
  if (otaCheckTaskRunning || otaUpdateInProgress) return; // предыдущая ещё идёт

  otaCheckTaskRunning = true;
  otaCheckResultReady = false;
  // стек 12 КБ: TLS-рукопожатие + HTTPClient (loopTask с 8 КБ справлялся, здесь запас)
  if (xTaskCreatePinnedToCore(otaCheckTask, "otaCheck", 12288, NULL, 1, NULL, 0) != pdPASS) {
    otaCheckTaskRunning = false; // не хватило памяти - попробуем в следующие сутки
  }
}

// Вызывается из loop(): если фоновая проверка закончилась - применяет результат
// (баннер "доступно обновление"). Работа с EEPROM (ранее отклонённая версия)
// и со String выполняется ТОЛЬКО здесь, в потоке loop().
void processOtaCheckResult() {
  if (!otaCheckResultReady) return;
  otaCheckResultReady = false;
  diagEvent(EV_OTA_CHECK, otaCheckResultOk ? 1 : 2, (int32_t)diagOtaCheckLastMs);
  if (!otaCheckResultOk) return; // как и раньше: при неудаче баннер не меняем

  String remoteVersion = String(otaCheckRemoteVersion);
  if (isVersionNewer(remoteVersion, FIRMWARE_VERSION)) {
    String declined = loadOtaDeclinedVersion();
    if (declined != remoteVersion) {
      otaUpdateAvailable = true;
      otaAvailableVersion = remoteVersion;
    }
  } else {
    otaUpdateAvailable = false;
  }
}

// Вызывается каждую итерацию loop() - раз в сутки запускает проверку (см.
// checkForFirmwareUpdate()), плюс один раз сразу после первого подключения
// к Wi-Fi, не дожидаясь первых суток.
void updateOtaCheck() {
  processOtaCheckResult(); // забрать результат фоновой проверки, если она закончилась
  if (wifiState != WIFI_ST_CONNECTED) return;

  unsigned long now = millis();
  if (!otaCheckedOnce || (now - lastOtaCheckMs >= OTA_CHECK_INTERVAL_MS)) {
    lastOtaCheckMs = now;
    otaCheckedOnce = true;
    checkForFirmwareUpdate();
  }
}

// Собственно скачивание и запись новой прошивки - вызывается из loop() по
// флагу otaConfirmedTriggered (не прямо из веб-обработчика, чтобы сначала
// успеть отправить ответ браузеру, а уже потом начинать блокирующую
// операцию). БЛОКИРУЕТ выполнение на время скачивания и записи (обычно
// 10-30 секунд в зависимости от размера прошивки и скорости сети) - это
// осознанный компромисс для операции, которую пользователь запускает сам и
// сознательно; после перезагрузки уже отработанная логика безопасного
// холодного старта (SSR_STARTUP_DELAY_MS, EMR_STARTUP_DELAY_MS) сама
// прикроет переходный период.
void performFirmwareUpdate() {
  otaUpdateInProgress = true;

  // Скачивание блокирует loop() на десятки секунд - в это время ни ток, ни
  // температура не контролируются. Поэтому нагрев на время обновления
  // принудительно выключаем (термостат сам включит его обратно при необходимости
  // после возврата управления в loop()), а watchdog приостанавливаем.
  ssrState = false;
  digitalWrite(PIN_SSR, SSR_INACTIVE_LEVEL);
  lastSsrTransitionMs = millis();
  WdtPause wdtPause;
  DiagBusy diagBusy(STAGE_OTA_DOWNLOAD);

  WiFiClientSecure client;
  client.setInsecure(); // см. пояснение в otaCheckTask()
  client.setHandshakeTimeout(10); // раньше таймаутов не было совсем: обрыв связи мог повесить скачивание

  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(15000); // таймаут простоя потока (не общий) - медленная, но живая загрузка не прервётся
  if (!http.begin(client, OTA_FIRMWARE_URL)) {
    otaUpdateInProgress = false;
    return;
  }

  int httpCode = http.GET();
  bool success = false;

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    if (contentLength > 0 && Update.begin(contentLength)) {
      WiFiClient *stream = http.getStreamPtr();
      size_t written = Update.writeStream(*stream);
      if (written == (size_t)contentLength && Update.end() && Update.isFinished()) {
        success = true;
      }
    }
  }

  http.end();

  if (success) {
    diagBusyClear(); // перезапуск здесь штатный, не зависание
    ESP.restart(); // новая прошивка вступит в силу после перезагрузки
    return;         // сюда управление не дойдёт - для ясности кода
  }

  // не получилось (сеть оборвалась, битый файл, нехватка места и т.п.) -
  // просто остаёмся на текущей рабочей прошивке
  otaUpdateInProgress = false;
}

// ---------------------------------------------------------------------------
// NTP - синхронизация реального времени (неблокирующая)
// ---------------------------------------------------------------------------

// Запускает (или перезапускает) синхронизацию - сам вызов НЕ блокирует,
// он только настраивает фоновый SNTP-клиент; реальная синхронизация
// происходит в фоне, готовность проверяется отдельно через isTimeSynced().
void requestTimeSync() {
  configTzTime(TZ_UKRAINE, NTP_SERVER_1, NTP_SERVER_2);
  lastTimeSyncMs = millis();
}

// Определяет, реально ли уже получено время, БЕЗ блокировки (в отличие от
// getLocalTime() с ненулевым таймаутом, который зависает в ожидании).
// Пока синхронизация не прошла, системные часы ESP32 показывают время
// около начала эпохи - проверка "после разумной даты" отличает
// синхронизированное время от ещё не установленного.
bool isTimeSynced() {
  time_t now = time(nullptr);
  return now > 1700000000; // примерно после ноября 2023 - явный признак реальной синхронизации
}

// Вызывается каждую итерацию loop(). Запускает первую попытку синхронизации,
// как только появился Wi-Fi, и затем повторяет её раз в сутки (см.
// TIME_RESYNC_INTERVAL_MS) - в том числе как естественный способ досинхронизироваться,
// если в момент первой попытки интернета не было, а позже появился.
void updateTimeSync() {
  if (wifiState != WIFI_ST_CONNECTED) {
    return; // без Wi-Fi пытаться нет смысла
  }

  if (!timeSyncRequested || (millis() - lastTimeSyncMs >= TIME_RESYNC_INTERVAL_MS)) {
    requestTimeSync();
    timeSyncRequested = true;
  }

  timeSynced = isTimeSynced();
}

// ---------------------------------------------------------------------------
// ВЕБ-СЕРВЕР
// ---------------------------------------------------------------------------

// Форматирование значений в строку БЕЗ float - та же логика "целая.дробная",
// что и на экране, просто собранная в строку для HTML/JSON вместо tft.print().
String formatTenths(int16_t tenths) {
  if (tenths == -1270) return "--.-";
  int16_t whole = tenths / 10;
  int16_t frac = tenths % 10;
  if (frac < 0) frac = -frac;
  return String(whole) + "." + String(frac);
}

String formatHundredths(int16_t hundredths) {
  int16_t whole = hundredths / 100;
  int16_t frac = hundredths % 100;
  if (frac < 0) frac = -frac;
  String fracStr = String(frac);
  if (frac < 10) fracStr = "0" + fracStr; // ведущий ноль для двух знаков после запятой
  return String(whole) + "." + fracStr;
}

// ---------------------------------------------------------------------------
// ИСТОРИЯ ДЛЯ ГРАФИКОВ (24 часа, в оперативной памяти, без SD/файловой системы)
// ---------------------------------------------------------------------------
// Кольцевой буфер: точка раз в минуту, 1440 точек = 24 часа. Абсолютное
// время не хранится (нет RTC/NTP) - ось времени на графике строится клиентом
// просто по номеру точки и известному интервалу (см. "intervalSec" в JSON).
// Память: 4 массива x 1440 x 2 байта = ~11.5 КБ - комфортно для ESP32.
const unsigned long HISTORY_SAMPLE_INTERVAL_MS = 60000UL; // 1 минута
const uint16_t HISTORY_SIZE = 1440;                        // 24 часа при интервале 1 минута

int16_t historyOut[HISTORY_SIZE];
int16_t historyAir[HISTORY_SIZE];
int16_t historyMat[HISTORY_SIZE];
int16_t historyCurrentMa[HISTORY_SIZE];

uint16_t historyWriteIndex = 0;   // куда писать следующую точку (кольцевой указатель)
uint16_t historyFilledCount = 0;  // сколько точек реально заполнено (растёт до HISTORY_SIZE, потом остаётся максимумом)

// Сквозной номер записанных точек с момента запуска (не обнуляется при
// переполнении кольцевого буфера) + идентификатор запуска. По ним страница
// графиков запрашивает только НОВЫЕ точки (?since=...&boot=...), а не всю
// историю целиком каждые 20 секунд.
uint32_t historyTotalSamples = 0;
uint32_t historyBootId = 0; // выставляется в setup()

unsigned long lastHistorySampleMs = 0;

// --- НЕДЕЛЬНАЯ ИСТОРИЯ (7 суток, тоже только в оперативной памяти) ---
// Второй, более грубый кольцевой буфер: каждая точка - СРЕДНЕЕ из
// WEEK_STEP_SAMPLES минутных точек (10 минут), 1008 точек = 7 суток.
// Память: 4 массива x 1008 x 2 байта = ~8 КБ; ответ страницы на неделю тоже
// ~8 КБ (в пределах 16 КБ на запрос). При перезагрузке контроллера эта
// история, как и суточная, обнуляется - это сознательное решение.
const uint16_t WEEK_STEP_SAMPLES = 10;   // сколько минутных точек усредняется в одну недельную
const uint16_t WEEK_SIZE = 1008;         // 7 суток x 24 часа x 6 точек в час

int16_t weekOut[WEEK_SIZE];
int16_t weekAir[WEEK_SIZE];
int16_t weekMat[WEEK_SIZE];
int16_t weekCurrentMa[WEEK_SIZE];

uint16_t weekWriteIndex = 0;
uint16_t weekFilledCount = 0;
uint32_t weekTotalSamples = 0;

// накопители текущей (ещё не завершённой) недельной точки
uint16_t weekAccSamples = 0;                 // сколько минутных точек уже накоплено
int32_t  weekAccSumOut = 0, weekAccSumAir = 0, weekAccSumMat = 0, weekAccSumCur = 0;
uint16_t weekAccCntOut = 0, weekAccCntAir = 0, weekAccCntMat = 0; // сколько из них с ДЕЙСТВИТЕЛЬНЫМИ данными датчика

// Среднее с округлением до ближайшего целого; если действительных значений
// не было вовсе - "нет данных" (-1270, тот же код, что и в основном буфере).
int16_t weekAverage(int32_t sum, uint16_t cnt) {
  if (cnt == 0) return -1270;
  int32_t half = cnt / 2;
  return (int16_t)((sum >= 0) ? (sum + half) / cnt : (sum - half) / cnt);
}

// Добавляет очередную минутную точку в накопитель; когда накоплено
// WEEK_STEP_SAMPLES штук - записывает их среднее как одну недельную точку.
void accumulateWeekSample(int16_t out, int16_t air, int16_t mat, int16_t currentMa) {
  if (out != -1270) { weekAccSumOut += out; weekAccCntOut++; }
  if (air != -1270) { weekAccSumAir += air; weekAccCntAir++; }
  if (mat != -1270) { weekAccSumMat += mat; weekAccCntMat++; }
  weekAccSumCur += currentMa;
  weekAccSamples++;

  if (weekAccSamples < WEEK_STEP_SAMPLES) return;

  weekOut[weekWriteIndex]       = weekAverage(weekAccSumOut, weekAccCntOut);
  weekAir[weekWriteIndex]       = weekAverage(weekAccSumAir, weekAccCntAir);
  weekMat[weekWriteIndex]       = weekAverage(weekAccSumMat, weekAccCntMat);
  weekCurrentMa[weekWriteIndex] = weekAverage(weekAccSumCur, weekAccSamples); // ток всегда "действителен"

  weekWriteIndex = (weekWriteIndex + 1) % WEEK_SIZE;
  if (weekFilledCount < WEEK_SIZE) weekFilledCount++;
  weekTotalSamples++;

  weekAccSamples = 0;
  weekAccSumOut = weekAccSumAir = weekAccSumMat = weekAccSumCur = 0;
  weekAccCntOut = weekAccCntAir = weekAccCntMat = 0;
}

void recordHistorySample() {
  int16_t currentMa = (int16_t)((int32_t)currentAmpsForDisplay * 10); // сотые А -> мА

  historyTotalSamples++;
  historyOut[historyWriteIndex] = tempOut;
  historyAir[historyWriteIndex] = tempAir;
  historyMat[historyWriteIndex] = tempMat;
  historyCurrentMa[historyWriteIndex] = currentMa;

  historyWriteIndex = (historyWriteIndex + 1) % HISTORY_SIZE;
  if (historyFilledCount < HISTORY_SIZE) historyFilledCount++;

  accumulateWeekSample(tempOut, tempAir, tempMat, currentMa);
}

// Главная страница - вся вёрстка/стили/скрипт зашиты в один HTML (без внешних
// файлов и CDN), хранится в PROGMEM, чтобы не расходовать оперативную память
// постоянно. Адаптивна для десктопа и Android-телефона (viewport + гибкая
// вёрстка, без фиксированных пиксельных ширин).
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Будка</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  .card { background:#222; border-radius:12px; padding:16px; margin-bottom:14px; max-width:420px; margin-left:auto; margin-right:auto; }
  .row { display:flex; justify-content:space-between; padding:7px 0; font-size:19px; border-bottom:1px solid #333; }
  .row:last-child { border-bottom:none; }
  .label-blue { color:#4da6ff; }
  .label-white { color:#fff; }
  .label-red { color:#ff5c5c; }
  .label-orange { color:#ffa64d; }
  .v-blue { color:#4da6ff; font-weight:bold; }
  .v-white { color:#fff; font-weight:bold; }
  .v-red { color:#ff5c5c; font-weight:bold; }
  .v-orange { color:#ffa64d; font-weight:bold; }
  .status { font-size:19px; margin:8px 0 0 0; text-align:center; padding:8px; border-radius:8px; background:#333; font-weight:bold; }
  .status-on { color:#ff5c5c; }
  .status-off { color:#2ecc71; }
  .alarm { color:#111; background:#ffcc00; font-size:15px; margin:5px 0; padding:6px 8px; border-radius:6px; font-weight:bold; }
  .noalarm { color:#2ecc71; text-align:center; padding:4px; font-weight:bold; }
  input[type=range] { width:100%; height:38px; margin:14px 0 6px 0; }
  button { width:100%; padding:16px; font-size:19px; background:#2a6df5; color:#fff; border:none; border-radius:10px; margin-top:4px; }
  button.pending { background:#2ecc71; }
  button:active { background:#1e54c4; }
  button.secondary { background:#444; }
  button.secondary:active { background:#333; }
  .title { background:#3a3a3a; margin:-16px -16px 14px -16px; padding:16px; text-align:center; border-radius:12px 12px 0 0; font-size:26px; color:#fff; font-weight:bold; }
  .subtitle { margin:0 0 10px 0; font-size:20px; }
  #setpointVal { font-size:32px; text-align:center; margin:4px 0; font-weight:bold; }
  .ota-banner { background:#2a4d3a; border:1px solid #2ecc71; border-radius:10px; padding:14px; margin-bottom:14px; max-width:420px; margin-left:auto; margin-right:auto; display:none; }
  .ota-banner.show { display:block; }
  .ota-banner .ota-text { font-size:16px; margin-bottom:10px; text-align:center; }
  .ota-row { display:flex; gap:8px; }
  .ota-row button { margin-top:0; }
  .ota-decline { background:#555; }
  .ota-decline:active { background:#444; }
</style>
</head>
<body>
  <div class="ota-banner" id="otaBanner">
    <div class="ota-text" id="otaText">Доступна новая версия</div>
    <div class="ota-row">
      <button onclick="confirmOta()">Обновить</button>
      <button class="ota-decline" onclick="declineOta()">Отклонить</button>
    </div>
  </div>
  <div class="card">
    <div class="title">Будка</div>
    <div class="row"><span class="label-blue">Снаружи:</span><span id="outside" class="v-blue">--.-</span></div>
    <div class="row"><span class="label-white">Внутри:</span><span id="air" class="v-white">--.-</span></div>
    <div class="row"><span class="label-red">Внизу:</span><span id="mat" class="v-red">--.-</span></div>
    <div class="row"><span class="label-orange">Ток нагревателя:</span><span id="current" class="v-orange">--.-</span></div>
    <div class="status" id="heating">Нагрев: ---</div>
  </div>
  <div class="card">
    <div id="alarmsBlock"><div class="noalarm">...</div></div>
  </div>
  <div class="card">
    <h2 class="subtitle">Заданная температура</h2>
    <div id="setpointVal">--&deg;C</div>
    <input type="range" id="slider" min="5" max="30" value="16" step="1">
    <button id="applyBtn" onclick="applySetpoint()">Применить</button>
    <button class="secondary" onclick="location.href='/graphs'">Графики</button>
  </div>
<script>
let pendingSetpoint = null;
let applyTimeoutHandle = null;
const APPLY_TIMEOUT_MS = 10000;

function updateStatus() {
  fetch('/api/status').then(function(r){ return r.json(); }).then(function(d){
    document.getElementById('outside').textContent = d.outside + (d.outsideError ? ' ?' : '');
    document.getElementById('air').textContent = d.air + (d.airError ? ' ?' : '');
    document.getElementById('mat').textContent = d.mat + (d.matError ? ' ?' : '');
    document.getElementById('current').textContent = d.current + ' А';
    let heatingEl = document.getElementById('heating');
    heatingEl.textContent = 'Нагрев: ' + (d.heating ? 'ВКЛ' : 'ВЫКЛ') + (d.emergency ? ' (АВАРИЙНЫЙ РЕЖИМ)' : '');
    heatingEl.classList.toggle('status-on', d.heating);
    heatingEl.classList.toggle('status-off', !d.heating);

    let alarmsBlock = document.getElementById('alarmsBlock');
    if (d.alarms.length === 0) {
      alarmsBlock.innerHTML = '<div class="noalarm">Аварий нет</div>';
    } else {
      let html = '';
      for (let i = 0; i < d.alarms.length; i++) {
        html += '<div class="alarm">' + d.alarms[i] + '</div>';
      }
      alarmsBlock.innerHTML = html;
    }

    if (pendingSetpoint === null) {
      document.getElementById('slider').value = d.setpoint;
      document.getElementById('setpointVal').textContent = d.setpoint + '\u00B0C';
    }

    let otaBanner = document.getElementById('otaBanner');
    let otaText = document.getElementById('otaText');
    if (d.otaInProgress) {
      otaBanner.classList.add('show');
      otaText.textContent = 'Идёт обновление... устройство перезагрузится';
      otaBanner.querySelectorAll('button').forEach(function(b) { b.disabled = true; });
    } else if (d.otaAvailable) {
      otaBanner.classList.add('show');
      otaText.textContent = 'Доступна версия ' + d.otaVersion + ' (сейчас: ' + d.otaCurrentVersion + ')';
      otaBanner.querySelectorAll('button').forEach(function(b) { b.disabled = false; });
    } else {
      otaBanner.classList.remove('show');
    }
  });
}

function confirmOta() {
  // ВАЖНО: после этого запроса контроллер сразу переходит к скачиванию и
  // прошивке новой версии - это БЛОКИРУЮЩАЯ операция (см. performFirmwareUpdate()
  // в прошивке), и пока она идёт, устройство физически не может отвечать ни
  // на какие HTTP-запросы, включая обычный опрос /api/status. Поэтому не
  // ждём ответа сервера, а сразу показываем уведомление здесь же, на
  // клиенте, иначе страница будет просто выглядеть "зависшей".
  document.getElementById('otaText').textContent =
    'Идёт обновление прошивки... контроллер перезагрузится, страница обновится автоматически';
  document.getElementById('otaBanner').querySelectorAll('button').forEach(function(b) { b.disabled = true; });

  clearInterval(statusPollHandle); // обычный опрос бессмыслен, пока сервер занят/перезагружается

  fetch('/api/ota/confirm', { method: 'POST' }).catch(function() {});

  // ждём, пока контроллер закончит прошивку и перезагрузится, и периодически
  // пробуем достучаться до него - как только ответит, перезагружаем страницу
  // (она подхватит уже новую версию через /api/status)
  setTimeout(function retryAfterReboot() {
    fetch('/api/status').then(function() { location.reload(); })
      .catch(function() { setTimeout(retryAfterReboot, 3000); });
  }, 15000);
}

function declineOta() {
  fetch('/api/ota/decline', { method: 'POST' }).then(function() {
    document.getElementById('otaBanner').classList.remove('show');
  });
}

document.getElementById('slider').addEventListener('input', function() {
  pendingSetpoint = this.value;
  document.getElementById('setpointVal').textContent = this.value + '\u00B0C';
  document.getElementById('applyBtn').classList.add('pending');

  if (applyTimeoutHandle) clearTimeout(applyTimeoutHandle);
  applyTimeoutHandle = setTimeout(function() {
    // кнопка не нажата за отведённое время - откатываем к текущему значению с сервера
    pendingSetpoint = null;
    document.getElementById('applyBtn').classList.remove('pending');
    applyTimeoutHandle = null;
    updateStatus();
  }, APPLY_TIMEOUT_MS);
});

function applySetpoint() {
  let val = document.getElementById('slider').value;
  fetch('/api/setpoint?value=' + val, { method: 'POST' }).then(function(){
    pendingSetpoint = null;
    document.getElementById('applyBtn').classList.remove('pending');
    if (applyTimeoutHandle) {
      clearTimeout(applyTimeoutHandle);
      applyTimeoutHandle = null;
    }
  });
}

let statusPollHandle = setInterval(updateStatus, 3000);
updateStatus();
</script>
</body>
</html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiStatus() {
  const char* alarmTexts[6] = {
    "Перегрев коврика", "Превышен ток", "SSR залип",
    "Обрыв нагрузки", "Отказ датчика воздуха", "Отказ датчика коврика"
  };
  uint8_t alarmBits[6] = {
    ALARM_MAT_OVERHEAT, ALARM_OVERCURRENT, ALARM_SSR_STUCK,
    ALARM_OPEN_LOAD, ALARM_AIR_SENSOR_FAIL, ALARM_MAT_SENSOR_FAIL
  };

  String alarmsJson = "[";
  bool firstAlarm = true;
  for (uint8_t i = 0; i < 6; i++) {
    if (alarmFlags & alarmBits[i]) {
      if (!firstAlarm) alarmsJson += ",";
      alarmsJson += "\"";
      alarmsJson += alarmTexts[i];
      alarmsJson += "\"";
      firstAlarm = false;
    }
  }
  alarmsJson += "]";

  String json = "{";
  json += "\"outside\":\"" + formatTenths(tempOut) + "\",";
  json += "\"outsideError\":" + String(tempOutError ? "true" : "false") + ",";
  json += "\"air\":\"" + formatTenths(tempAir) + "\",";
  json += "\"airError\":" + String(tempAirError ? "true" : "false") + ",";
  json += "\"mat\":\"" + formatTenths(tempMat) + "\",";
  json += "\"matError\":" + String(tempMatError ? "true" : "false") + ",";
  json += "\"setpoint\":" + String(setpointC) + ",";
  json += "\"heating\":" + String(ssrState ? "true" : "false") + ",";
  json += "\"emergency\":" + String(emergencyModeActive ? "true" : "false") + ",";
  json += "\"current\":\"" + formatHundredths(currentAmpsForDisplay) + "\",";
  json += "\"alarms\":" + alarmsJson + ",";
  json += "\"otaAvailable\":" + String(otaUpdateAvailable ? "true" : "false") + ",";
  json += "\"otaVersion\":\"" + otaAvailableVersion + "\",";
  json += "\"otaCurrentVersion\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"otaInProgress\":" + String(otaUpdateInProgress ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleApiSetpoint() {
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    if (val >= SETPOINT_MIN_C && val <= SETPOINT_MAX_C) {
      setpointC = (int8_t)val;
      pendingSetpointC = (int8_t)val; // держим в согласии с энкодером тоже
      saveSetpointToEeprom(setpointC);
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad value");
}

// Подтверждение обновления с веб-страницы - НЕ запускает скачивание прямо
// здесь, а только выставляет флаг: сама (блокирующая) операция запускается
// из loop() на следующей итерации, чтобы сначала успеть корректно отправить
// ответ браузеру, а уже потом начинать долгую блокирующую операцию.
void handleApiOtaConfirm() {
  if (!otaUpdateAvailable || otaUpdateInProgress) {
    server.send(409, "text/plain", "No update available or already in progress");
    return;
  }
  otaConfirmedTriggered = true;
  server.send(200, "text/plain", "OK, updating...");
}

void handleApiOtaDecline() {
  if (otaUpdateAvailable) {
    saveOtaDeclinedVersion(otaAvailableVersion);
    otaUpdateAvailable = false;
  }
  server.send(200, "text/plain", "OK");
}

// ---------------------------------------------------------------------------
// БИНАРНЫЙ ФОРМАТ ОТВЕТА /api/history
// ---------------------------------------------------------------------------
// Вместо JSON (4-6 символов на значение, до ~33 КБ на сутки) отдаём сырые
// int16 - 2 байта на значение. Полный суточный буфер (4 канала x 1440 точек)
// всегда занимает ровно 32 + 4 x 1440 x 2 = 11 552 байта, то есть укладывается
// в 16 КБ независимо от значений в данных; размер известен заранее, поэтому
// ответ уходит с обычным Content-Length (без chunked).
//
// Структура ответа (все числа little-endian, как в памяти ESP32):
//   [0..31]  заголовок HistoryBinHeader: 8 x uint32
//   далее    count x int16: температура "улица" (десятые доли °C, -1270 = нет данных)
//            count x int16: температура "воздух"
//            count x int16: температура "коврик"
//            count x int16: ток в мА
// Точки в каждом массиве идут в хронологическом порядке (старая -> новая).
// Тот же формат разбирается в JS на странице графиков (см. updateGraphs()).
struct __attribute__((packed)) HistoryBinHeader {
  uint32_t total;       // сквозной номер последней записанной точки (historyTotalSamples)
  uint32_t boot;        // идентификатор запуска контроллера (historyBootId)
  uint32_t count;       // сколько точек в каждом из четырёх массивов
  uint32_t flags;       // бит 0 - "full" (клиент должен заменить всё накопленное), бит 1 - время синхронизировано по NTP
  uint32_t nowEpoch;    // текущее время (unix), если timeSynced
  uint32_t intervalSec; // интервал между точками, секунд
  uint32_t maxCount;    // ёмкость буфера (HISTORY_SIZE)
  uint32_t reserved;    // запас на будущее, всегда 0
};
static_assert(sizeof(HistoryBinHeader) == 32, "HistoryBinHeader must be exactly 32 bytes");

// Отправляет один канал истории, разворачивая кольцевой буфер в
// хронологический порядок: сначала кусок от startIndex до конца массива,
// затем (если буфер "перешагнул" границу) продолжение с начала массива.
void sendHistoryChannel(const int16_t* data, uint16_t count, uint16_t startIndex, uint16_t ringSize) {
  uint16_t firstPart = ringSize - startIndex;
  if (firstPart > count) firstPart = count;
  if (firstPart > 0) {
    server.sendContent((const char*)&data[startIndex], firstPart * sizeof(int16_t));
  }
  uint16_t secondPart = count - firstPart;
  if (secondPart > 0) {
    server.sendContent((const char*)&data[0], secondPart * sizeof(int16_t));
  }
}

// Описание одного кольцевого буфера истории (суточного или недельного) -
// обработчик ниже работает с любым из них одинаково.
struct HistoryRingView {
  const int16_t* out;
  const int16_t* air;
  const int16_t* mat;
  const int16_t* currentMa;
  uint16_t ringSize;
  uint16_t writeIndex;
  uint16_t filled;
  uint32_t total;
  uint32_t intervalSec;
};

// Параметры запроса (все необязательны):
//   range - "week" для недельной истории (точки по 10 минут), иначе суточная (по 1 минуте)
//   since - сквозной номер последней точки, которую клиент уже получил
//   boot  - идентификатор запуска контроллера, при котором клиент получал точки
// Если клиент передал актуальные since и boot, отдаются ТОЛЬКО новые точки
// (флаг full = 0). Иначе (первый запрос, перезагрузка контроллера, клиент
// слишком давно не обновлялся и часть точек уже вытеснена) - вся история
// (флаг full = 1). Нумерация since у суточного и недельного буферов своя.
void handleApiHistory() {
  uint32_t since = 0;
  uint32_t clientBoot = 0;
  if (server.hasArg("since")) since = strtoul(server.arg("since").c_str(), NULL, 10);
  if (server.hasArg("boot"))  clientBoot = strtoul(server.arg("boot").c_str(), NULL, 10);

  HistoryRingView ring;
  if (server.hasArg("range") && server.arg("range") == "week") {
    ring.out = weekOut;  ring.air = weekAir;  ring.mat = weekMat;  ring.currentMa = weekCurrentMa;
    ring.ringSize   = WEEK_SIZE;
    ring.writeIndex = weekWriteIndex;
    ring.filled     = weekFilledCount;
    ring.total      = weekTotalSamples;
    ring.intervalSec = (uint32_t)(WEEK_STEP_SAMPLES * (HISTORY_SAMPLE_INTERVAL_MS / 1000));
  } else {
    ring.out = historyOut;  ring.air = historyAir;  ring.mat = historyMat;  ring.currentMa = historyCurrentMa;
    ring.ringSize   = HISTORY_SIZE;
    ring.writeIndex = historyWriteIndex;
    ring.filled     = historyFilledCount;
    ring.total      = historyTotalSamples;
    ring.intervalSec = (uint32_t)(HISTORY_SAMPLE_INTERVAL_MS / 1000);
  }

  uint16_t filled = ring.filled;
  uint32_t total = ring.total;

  bool full = true;
  uint16_t count = filled;
  if (since > 0 && clientBoot == historyBootId && since <= total && (total - since) <= filled) {
    full = false;
    count = (uint16_t)(total - since);
  }

  // индекс самой старой из отправляемых точек в кольцевом буфере
  uint16_t startIndex = (ring.writeIndex + ring.ringSize - count) % ring.ringSize;

  HistoryBinHeader head;
  head.total       = total;
  head.boot        = historyBootId;
  head.count       = count;
  head.flags       = (full ? 1u : 0u) | (timeSynced ? 2u : 0u);
  head.nowEpoch    = (uint32_t)time(nullptr);
  head.intervalSec = ring.intervalSec;
  head.maxCount    = ring.ringSize;
  head.reserved    = 0;

  // размер известен заранее: заголовок + 4 канала по count значений int16
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(sizeof(HistoryBinHeader) + 4u * count * sizeof(int16_t));
  server.send(200, "application/octet-stream", "");

  server.sendContent((const char*)&head, sizeof(head));
  sendHistoryChannel(ring.out,       count, startIndex, ring.ringSize);
  sendHistoryChannel(ring.air,       count, startIndex, ring.ringSize);
  sendHistoryChannel(ring.mat,       count, startIndex, ring.ringSize);
  sendHistoryChannel(ring.currentMa, count, startIndex, ring.ringSize);
}

// Страница графиков - температуры (в градусах, с точностью 0.1) и ток
// (в миллиамперах) за последние 24 часа. Рисуется на <canvas> обычным JS,
// без сторонних библиотек - страница работает даже без доступа в интернет
// у браузера, что соответствует общей идее "только локальная сеть".
const char GRAPHS_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Будка - графики</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  .card { background:#222; border-radius:12px; padding:16px; margin-bottom:14px; max-width:420px; margin-left:auto; margin-right:auto; }
  .title { background:#3a3a3a; margin:-16px -16px 14px -16px; padding:16px; text-align:center; border-radius:12px 12px 0 0; font-size:26px; color:#fff; font-weight:bold; }
  .subtitle { margin:0 0 8px 0; font-size:17px; color:#ccc; }
  .legend { font-size:13px; margin-bottom:8px; }
  .legend span { margin-right:14px; }
  .dot { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:4px; }
  canvas { width:100%; height:auto; display:block; background:#181818; border-radius:8px; }
  #tempChart { cursor:crosshair; touch-action:none; }
  .info { font-size:12px; color:#777; text-align:center; margin-top:8px; }
  button { width:100%; padding:16px; font-size:19px; background:#444; color:#fff; border:none; border-radius:10px; margin-top:4px; }
  button:active { background:#333; }
  .card button + button { margin-top:10px; }
  .seg { display:flex; gap:8px; margin-bottom:12px; }
  .seg button { flex:1; padding:10px; font-size:16px; margin-top:0; background:#333; }
  .seg button.on { background:#2a6df5; }
</style>
</head>
<body>
  <div class="card">
    <div class="title">Будка</div>
    <div class="seg">
      <button id="btnDay" class="on" onclick="setRange('day')">24 часа</button>
      <button id="btnWeek" onclick="setRange('week')">7 дней</button>
    </div>
    <h2 class="subtitle">Температуры, &deg;C (<span class="rangeLabel">24 часа</span>)</h2>
    <div class="legend">
      <span><span class="dot" style="background:#4da6ff"></span>Снаружи</span>
      <span><span class="dot" style="background:#fff"></span>Внутри</span>
      <span><span class="dot" style="background:#ff5c5c"></span>Внизу</span>
    </div>
    <canvas id="tempChart" width="600" height="220"></canvas>
  </div>
  <div class="card">
    <h2 class="subtitle">Ток нагревателя, мА (<span class="rangeLabel">24 часа</span>)</h2>
    <div class="legend">
      <span><span class="dot" style="background:#ffa64d"></span>Ток</span>
    </div>
    <canvas id="curChart" width="600" height="220"></canvas>
    <div class="info" id="pointsInfo">...</div>
  </div>
  <div class="card">
    <button onclick="location.href='/diag'">Диагностика</button>
    <button onclick="location.href='/'">Параметры</button>
  </div>
<script>
function drawChart(canvas, seriesList, maxSlots, forcePositive, timeSynced, nowEpoch, hoverX, showLastValueLabel, stepMinutes) {
  const step = stepMinutes || 1; // минут в одной точке (1 - сутки, 10 - неделя)
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  let minV = Infinity, maxV = -Infinity;
  let dataCount = 0;
  seriesList.forEach(function(s) {
    if (s.data.length > dataCount) dataCount = s.data.length;
    s.data.forEach(function(v) {
      if (v !== null) { if (v < minV) minV = v; if (v > maxV) maxV = v; }
    });
  });
  if (minV === Infinity) { minV = 0; maxV = 1; }
  if (minV === maxV) { minV -= 1; maxV += 1; }
  const pad = (maxV - minV) * 0.1;
  minV -= pad; maxV += pad;
  if (forcePositive && minV < 0) minV = 0; // ток отрицательным не бывает - шкала не должна уходить ниже нуля

  // --- окно по оси X = ФАКТИЧЕСКИ накопленные данные (минуты), не более
  // ёмкости буфера. Пока данных мало, график "зумируется" под них, показывая
  // всю ширину холста занятой; когда буфер заполнится полностью (maxSlots,
  // т.е. 24ч) - окно перестаёт расти, и новые точки начинают вытеснять
  // старые слева, что и даёт эффект прокрутки "как на осциллографе". ---
  const windowMinutes = Math.max(step, Math.min(dataCount, maxSlots) * step);

  // --- шаг сетки по времени подбирается под размер окна ---
  let gridStepMin;
  if (windowMinutes <= 60) gridStepMin = 10;
  else if (windowMinutes <= 120) gridStepMin = 15;
  else if (windowMinutes <= 240) gridStepMin = 30;
  else if (windowMinutes <= 2880) gridStepMin = 60;
  else gridStepMin = 1440;

  const marginLeft = 42, marginRight = 8, marginTop = 8, marginBottom = 16;
  const plotW = w - marginLeft - marginRight;
  const plotH = h - marginTop - marginBottom;

  // --- горизонтальная сетка + подписи оси Y ---
  ctx.strokeStyle = '#333';
  ctx.fillStyle = '#2ecc71';
  ctx.font = '13px sans-serif';
  ctx.lineWidth = 1;
  const gridLines = 4;
  for (let i = 0; i <= gridLines; i++) {
    const y = marginTop + plotH * i / gridLines;
    const val = maxV - (maxV - minV) * i / gridLines;
    ctx.beginPath();
    ctx.moveTo(marginLeft, y);
    ctx.lineTo(w - marginRight, y);
    ctx.stroke();
    ctx.fillText(val.toFixed(1), 2, y + 4);
  }

  // --- вертикальная сетка по времени ---
  ctx.strokeStyle = '#2a2a2a';
  ctx.fillStyle = '#2ecc71';
  ctx.font = '12px sans-serif';

  if (timeSynced && nowEpoch > 0 && windowMinutes > 2880) {
    // длинное окно (неделя): линии на полуночах, подписи - день недели и число.
    // Перебор через setDate(-1), а не "минус 86400 с", чтобы переход на
    // летнее/зимнее время не сдвигал линии.
    const dayNames = ['Вс', 'Пн', 'Вт', 'Ср', 'Чт', 'Пт', 'Сб'];
    const windowStartEpoch = nowEpoch - windowMinutes * 60;
    let day = new Date(nowEpoch * 1000);
    day.setHours(0, 0, 0, 0);
    for (let guard = 0; guard < 40; guard++) {
      const t = Math.floor(day.getTime() / 1000);
      if (t < windowStartEpoch) break;
      const x = marginLeft + plotW * (1 - ((nowEpoch - t) / 60) / windowMinutes);
      ctx.beginPath();
      ctx.moveTo(x, marginTop);
      ctx.lineTo(x, marginTop + plotH);
      ctx.stroke();
      const lbl = dayNames[day.getDay()] + ' ' + day.getDate();
      if (x + 2 + ctx.measureText(lbl).width <= w) ctx.fillText(lbl, x + 2, h - 3);
      day.setDate(day.getDate() - 1);
      day.setHours(0, 0, 0, 0);
    }
  } else if (timeSynced && nowEpoch > 0) {
    // реальное время известно (NTP) - линии рисуем СТРОГО на ровных часах
    // (минута = 00), а не через фиксированный шаг назад от "сейчас" - так
    // линия и её подпись всегда совпадают с началом часа на настенных часах.
    const windowStartEpoch = nowEpoch - windowMinutes * 60;

    // ближайшая ровная граница часа, которая не позже "сейчас"
    let boundary = new Date(nowEpoch * 1000);
    boundary.setMinutes(0, 0, 0);
    let boundaryEpoch = Math.floor(boundary.getTime() / 1000);
    if (boundaryEpoch > nowEpoch) boundaryEpoch -= 3600;

    for (let t = boundaryEpoch; t >= windowStartEpoch; t -= 3600) {
      const m = (nowEpoch - t) / 60; // минут назад от "сейчас"
      const x = marginLeft + plotW * (1 - m / windowMinutes);
      ctx.beginPath();
      ctx.moveTo(x, marginTop);
      ctx.lineTo(x, marginTop + plotH);
      ctx.stroke();

      // подпись - только целый час, без минут (они всегда "00")
      const hourLabel = String(new Date(t * 1000).getHours());
      ctx.fillText(hourLabel, Math.max(marginLeft, x - 6), h - 3);
    }
  } else {
    // NTP недоступен (нет интернета у Wi-Fi сети) - абсолютное время часов
    // неизвестно, поэтому оставляем относительную сетку, как раньше
    const totalLines = Math.floor(windowMinutes / gridStepMin);
    const labelEvery = Math.max(1, Math.round(totalLines / 6));
    for (let lineNum = 0; lineNum <= totalLines; lineNum++) {
      const m = lineNum * gridStepMin;
      const x = marginLeft + plotW * (1 - m / windowMinutes);
      ctx.beginPath();
      ctx.moveTo(x, marginTop);
      ctx.lineTo(x, marginTop + plotH);
      ctx.stroke();

      if (lineNum % labelEvery === 0) {
        let label;
        if (m === 0) label = 'now';
        else if (gridStepMin === 1440) label = '-' + (m / 1440) + 'd';
        else if (m < 60) label = '-' + m + 'm';
        else label = '-' + (m % 60 === 0 ? (m / 60) : (m / 60).toFixed(1)) + 'h';
        ctx.fillText(label, Math.max(marginLeft, x - 12), h - 3);
      }
    }
  }

  // --- сами линии данных: самая свежая точка всегда у правого края,
  // окно = windowMinutes (см. выше). dataCount уже посчитан выше (при
  // определении minV/maxV) - используем его же, повторно не объявляем. ---
  seriesList.forEach(function(s) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    let lastX = null, lastY = null, lastV = null;
    const n = s.data.length;
    for (let i = 0; i < n; i++) {
      const v = s.data[i];
      const distFromNewest = (n - 1 - i); // 0 = самая свежая точка
      const x = marginLeft + plotW * (1 - distFromNewest * step / windowMinutes);
      if (v === null) { started = false; continue; }
      const y = marginTop + plotH * (1 - (v - minV) / (maxV - minV));
      if (!started) { ctx.moveTo(x, y); started = true; }
      else { ctx.lineTo(x, y); }
      lastX = x; lastY = y; lastV = v;
    }
    ctx.stroke();

    // --- подпись значения ПОСЛЕДНЕЙ (самой правой, самой свежей) точки -
    // всегда видна без наведения, чтобы на телефоне не нужно было "тыкать"
    // в график ради текущего значения (опционально - см. showLastValueLabel) ---
    if (lastV !== null && showLastValueLabel !== false) {
      ctx.beginPath();
      ctx.arc(lastX, lastY, 3, 0, Math.PI * 2);
      ctx.fillStyle = s.color;
      ctx.fill();

      let labelY = lastY - 6;
      if (labelY < marginTop + 10) labelY = marginTop + 10;
      if (labelY > marginTop + plotH - 2) labelY = marginTop + plotH - 2;

      ctx.font = 'bold 12px sans-serif';
      ctx.textAlign = 'right';
      ctx.fillText(lastV.toFixed(s.decimals === undefined ? 1 : s.decimals) + (s.unit || ''), w - marginRight - 2, labelY);
      ctx.textAlign = 'left'; // возвращаем выравнивание по умолчанию для остальной отрисовки
    }
  });

  // --- курсор наведения (мышь) / касания (телефон): подпись реального
  // значения на момент логирования в точке под курсором/пальцем ---
  if (hoverX !== null && hoverX !== undefined && dataCount > 0) {
    const relX = Math.min(1, Math.max(0, (hoverX - marginLeft) / plotW));
    const distFromNewest = Math.round((1 - relX) * windowMinutes / step);
    const idx = dataCount - 1 - distFromNewest;

    if (idx >= 0 && idx < dataCount) {
      const cx = marginLeft + plotW * (1 - distFromNewest * step / windowMinutes);

      ctx.save();
      ctx.strokeStyle = '#999';
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(cx, marginTop);
      ctx.lineTo(cx, marginTop + plotH);
      ctx.stroke();
      ctx.restore();

      let timeLabel;
      if (timeSynced && nowEpoch > 0) {
        const ts = new Date((nowEpoch - distFromNewest * step * 60) * 1000);
        const hh = ('0' + ts.getHours()).slice(-2);
        const mm = ('0' + ts.getMinutes()).slice(-2);
        timeLabel = (step > 1 ? ts.getDate() + '.' + ('0' + (ts.getMonth() + 1)).slice(-2) + ' ' : '') + hh + ':' + mm;
      } else {
        timeLabel = distFromNewest === 0 ? 'сейчас' : ('-' + (distFromNewest * step) + ' мин');
      }

      const lines = [timeLabel];
      seriesList.forEach(function(s) {
        const v = s.data[idx];
        if (v === null || v === undefined) return;
        const y = marginTop + plotH * (1 - (v - minV) / (maxV - minV));
        ctx.beginPath();
        ctx.arc(cx, y, 4, 0, Math.PI * 2);
        ctx.fillStyle = s.color;
        ctx.fill();
        ctx.strokeStyle = '#111';
        ctx.lineWidth = 1;
        ctx.stroke();
        lines.push((s.label ? s.label + ': ' : '') + v.toFixed(s.decimals === undefined ? 1 : s.decimals) + (s.unit || ''));
      });

      ctx.font = '12px sans-serif';
      const lineHeight = 15;
      let boxW = 0;
      lines.forEach(function(t) { boxW = Math.max(boxW, ctx.measureText(t).width); });
      boxW += 12;
      const boxH = lines.length * lineHeight + 8;

      let boxX = cx + 8;
      if (boxX + boxW > w - marginRight) boxX = cx - boxW - 8;
      if (boxX < marginLeft) boxX = marginLeft;
      const boxY = marginTop + 4;

      ctx.fillStyle = 'rgba(20,20,20,0.92)';
      ctx.fillRect(boxX, boxY, boxW, boxH);
      ctx.strokeStyle = '#555';
      ctx.strokeRect(boxX, boxY, boxW, boxH);

      lines.forEach(function(t, i) {
        ctx.fillStyle = i === 0 ? '#ccc' : '#fff';
        ctx.fillText(t, boxX + 6, boxY + 14 + i * lineHeight);
      });
    }
  }
}

function conv10(arr) {
  return arr.map(function(v) { return v === -1270 ? null : v / 10; });
}

// последние полученные с сервера данные истории - хранятся отдельно от
// самой отрисовки, чтобы при наведении/касании можно было мгновенно
// перерисовать график (с курсором) БЕЗ повторного похода на сервер
let lastHistory = null;
let tempHoverX = null; // X-координата курсора/пальца в системе координат canvas (не CSS-пикселей)
// у графика тока наведение/курсор не используются - он остаётся как раньше

function redrawCharts() {
  if (!lastHistory) return;
  const d = lastHistory;

  drawChart(document.getElementById('tempChart'), [
    { data: conv10(d.out), color: '#4da6ff', label: 'Снаружи', unit: '°C', decimals: 1 },
    { data: conv10(d.air), color: '#fff', label: 'Внутри', unit: '°C', decimals: 1 },
    { data: conv10(d.mat), color: '#ff5c5c', label: 'Внизу', unit: '°C', decimals: 1 }
  ], d.maxCount, false, d.timeSynced, d.nowEpoch, tempHoverX, true, d.intervalSec / 60);

  drawChart(document.getElementById('curChart'), [
    { data: d.currentMa, color: '#ffa64d', label: 'Ток', unit: ' мА', decimals: 0 }
  ], d.maxCount, true, d.timeSynced, d.nowEpoch, null, false, d.intervalSec / 60);
}

// --- накопленная на клиенте история ---
// Для каждого диапазона ('day' - 24 ч по минутам, 'week' - 7 суток по 10-минутным
// средним) хранится своя копия. Первый запрос диапазона забирает всю историю,
// дальше запрашиваются только НОВЫЕ точки (since = сквозной номер последней
// полученной точки, boot = идентификатор запуска контроллера). Если контроллер
// перезагрузился или клиент отстал, он сам отвечает полной историей
// ("full":true), и мы её заменяем.
let currentRange = 'day';
const histStore = { day: null, week: null }; // { arrays, total, boot, view }
let fetchBusy = false, fetchQueued = false;

function appendTrim(dst, src, maxN) {
  for (let i = 0; i < src.length; i++) dst.push(src[i]);
  if (dst.length > maxN) dst.splice(0, dst.length - maxN);
}

// Разбор бинарного ответа /api/history (формат описан в прошивке рядом с
// HistoryBinHeader): 32 байта заголовка (8 x uint32) и затем четыре массива
// int16 по count значений - улица, воздух, коврик, ток. Всё little-endian.
// Читаем через DataView с явным указанием порядка байт, а не Int16Array,
// чтобы результат не зависел от архитектуры устройства, где открыта страница.
function parseHistoryBuffer(buf) {
  if (buf.byteLength < 32) throw new Error('short history response');
  const dv = new DataView(buf);
  const d = {
    total:       dv.getUint32(0, true),
    boot:        dv.getUint32(4, true),
    count:       dv.getUint32(8, true),
    full:        (dv.getUint32(12, true) & 1) !== 0,
    timeSynced:  (dv.getUint32(12, true) & 2) !== 0,
    nowEpoch:    dv.getUint32(16, true),
    intervalSec: dv.getUint32(20, true),
    maxCount:    dv.getUint32(24, true)
  };
  if (buf.byteLength !== 32 + d.count * 8) throw new Error('bad history response size');

  function readChannel(index) {
    const arr = new Array(d.count);
    const base = 32 + index * d.count * 2;
    for (let i = 0; i < d.count; i++) arr[i] = dv.getInt16(base + i * 2, true);
    return arr;
  }
  d.out = readChannel(0);
  d.air = readChannel(1);
  d.mat = readChannel(2);
  d.currentMa = readChannel(3);
  return d;
}

function updateInfo(view) {
  const totalHours = view.count * view.intervalSec / 3600;
  const span = totalHours >= 48 ? (totalHours / 24).toFixed(1) + ' сут' : totalHours.toFixed(1) + ' ч';
  const timeStatus = view.timeSynced ? 'время: NTP' : 'время: относительное (нет NTP)';
  document.getElementById('pointsInfo').textContent = view.count + ' точек (~' + span + '), интервал ' + view.intervalSec + ' с, ' + timeStatus;
}

function updateGraphs() {
  // запросы идут строго по одному: иначе два параллельных инкрементальных
  // запроса с одним и тем же since вернули бы одни и те же точки дважды
  if (fetchBusy) { fetchQueued = true; return; }
  fetchBusy = true;
  const range = currentRange;
  const st = histStore[range];
  let query = '?range=' + range;
  if (st) query += '&since=' + st.total + '&boot=' + st.boot;
  fetch('/api/history' + query).then(function(r){
    if (!r.ok) throw new Error('history HTTP ' + r.status);
    return r.arrayBuffer();
  }).then(function(buf){
    const d = parseHistoryBuffer(buf);
    let arrays;
    if (d.full || !st) {
      arrays = { out: d.out, air: d.air, mat: d.mat, currentMa: d.currentMa };
    } else {
      arrays = st.arrays;
      appendTrim(arrays.out, d.out, d.maxCount);
      appendTrim(arrays.air, d.air, d.maxCount);
      appendTrim(arrays.mat, d.mat, d.maxCount);
      appendTrim(arrays.currentMa, d.currentMa, d.maxCount);
    }
    const view = {
      out: arrays.out, air: arrays.air, mat: arrays.mat, currentMa: arrays.currentMa,
      maxCount: d.maxCount, intervalSec: d.intervalSec,
      timeSynced: d.timeSynced, nowEpoch: d.nowEpoch,
      count: arrays.out.length
    };
    histStore[range] = { arrays: arrays, total: d.total, boot: d.boot, view: view };
    if (range === currentRange) {
      lastHistory = view;
      redrawCharts();
      updateInfo(view);
    }
  }).catch(function(){ /* связь пропала - повторим на следующем тике */ }).then(function(){
    fetchBusy = false;
    if (fetchQueued) { fetchQueued = false; updateGraphs(); }
  });
}

function setRange(r) {
  if (r === currentRange) return;
  currentRange = r;
  document.getElementById('btnDay').className = (r === 'day') ? 'on' : '';
  document.getElementById('btnWeek').className = (r === 'week') ? 'on' : '';
  const labels = document.getElementsByClassName('rangeLabel');
  for (let i = 0; i < labels.length; i++) labels[i].textContent = (r === 'day') ? '24 часа' : '7 дней';
  const st = histStore[r];
  if (st) {
    lastHistory = st.view;
    redrawCharts();
    updateInfo(st.view);
  } else {
    lastHistory = null;
    ['tempChart', 'curChart'].forEach(function(id) {
      const c = document.getElementById(id);
      c.getContext('2d').clearRect(0, 0, c.width, c.height);
    });
    document.getElementById('pointsInfo').textContent = '...';
  }
  updateGraphs(); // сразу подтянуть свежие точки выбранного диапазона
}

// --- наведение мышью / касание пальцем: переводим координату события в
// систему координат canvas (её внутреннее разрешение 600x220 может
// отличаться от того, как она физически растянута на экране через CSS) и
// просто перерисовываем график с этой позицией - сами данные уже есть в
// lastHistory, повторный запрос к серверу не нужен ---
function attachHoverHandlers(canvas, setHoverX) {
  function canvasXFromClientX(clientX) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    return (clientX - rect.left) * scaleX;
  }

  canvas.addEventListener('mousemove', function(e) {
    setHoverX(canvasXFromClientX(e.clientX));
    redrawCharts();
  });
  canvas.addEventListener('mouseleave', function() {
    setHoverX(null);
    redrawCharts();
  });

  canvas.addEventListener('touchstart', function(e) {
    e.preventDefault(); // не даём странице скроллиться, пока палец на графике
    setHoverX(canvasXFromClientX(e.touches[0].clientX));
    redrawCharts();
  }, { passive: false });
  canvas.addEventListener('touchmove', function(e) {
    e.preventDefault();
    setHoverX(canvasXFromClientX(e.touches[0].clientX));
    redrawCharts();
  }, { passive: false });
  canvas.addEventListener('touchend', function() {
    setHoverX(null);
    redrawCharts();
  });
}

attachHoverHandlers(document.getElementById('tempChart'), function(x) { tempHoverX = x; });
// на графике тока наведение/курсор не нужны - обработчики не вешаем, canvas
// остаётся с CSS-курсором по умолчанию (свойство cursor:crosshair снято ниже
// точечно не требуется - карточка тока просто не реагирует на события)

// новая точка истории появляется раз в минуту (HISTORY_SAMPLE_INTERVAL_MS) -
// чаще опрашивать нет смысла
setInterval(updateGraphs, 60000);
updateGraphs();

</script>
</body>
</html>
)HTMLPAGE";

const char DIAG_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Будка - диагностика</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  .card { background:#222; border-radius:12px; padding:16px; margin-bottom:14px; max-width:420px; margin-left:auto; margin-right:auto; }
  .title { background:#3a3a3a; margin:-16px -16px 14px -16px; padding:16px; text-align:center; border-radius:12px 12px 0 0; font-size:26px; color:#fff; font-weight:bold; }
  .info { font-size:12px; color:#777; text-align:center; margin-top:8px; }
  button { width:100%; padding:16px; font-size:19px; background:#444; color:#fff; border:none; border-radius:10px; margin-top:4px; }
  button:active { background:#333; }
  .diag .kv { display:flex; justify-content:space-between; gap:12px; font-size:14px; padding:5px 0; border-bottom:1px solid #2c2c2c; }
  .diag .kv b { font-weight:normal; color:#aaa; }
  .diag .kv span { text-align:right; }
  .diag .ev { font-size:13px; color:#ccc; padding:7px 0; border-bottom:1px solid #2c2c2c; line-height:1.4; }
  .diag .ev strong { color:#fff; }
  .diag h3 { font-size:15px; color:#ccc; margin:14px 0 4px 0; }
</style>
</head>
<body>
  <div class="card">
    <div class="title">Диагностика</div>
    <div class="diag" id="diagBody">...</div>
    <div class="info">обновляется раз в 30 секунд</div>
  </div>
  <div class="card">
    <button onclick="location.href='/graphs'">Графики</button>
  </div>
<script>
// --- блок "Диагностика": причина последнего сброса, время работы, журнал
// перезагрузок (данные отдаёт /api/diag) ---
const STAGE_NAMES = ['основной цикл', 'кнопки/энкодер', 'Wi-Fi/время', 'проверка версии OTA',
  'веб-сервер', 'скачивание OTA', 'опрос датчиков', 'датчик тока', 'термостат', 'экран', 'запуск (setup)'];
const REASON_NAMES = {
  0: 'неизвестна', 1: 'включение питания', 2: 'сброс по EN', 3: 'программный перезапуск',
  4: 'сбой программы (паника)', 5: 'зависание прерываний (INT WDT)', 6: 'сторожевой таймер (TASK WDT)',
  7: 'сторожевой таймер (WDT)', 8: 'выход из сна', 9: 'просадка питания (BROWNOUT)', 10: 'SDIO',
  11: 'USB', 12: 'JTAG', 13: 'eFuse', 14: 'сбой питания (glitch)', 15: 'зависание процессора'
};
const WIFI_REASONS = {
  2: 'истекла авторизация', 8: 'точка отключила устройство', 15: 'тайм-аут рукопожатия',
  200: 'потеряны сигналы точки', 201: 'сеть не найдена', 202: 'ошибка аутентификации'
};

function fmtDur(sec) {
  if (sec < 60) return sec + ' с';
  const d = Math.floor(sec / 86400), h = Math.floor((sec % 86400) / 3600), m = Math.floor((sec % 3600) / 60);
  return (d ? d + ' д ' : '') + (d || h ? h + ' ч ' : '') + m + ' мин';
}
function fmtTime(epoch) { return new Date(epoch * 1000).toLocaleString('ru-RU'); }
function reasonName(c) { return REASON_NAMES[c] !== undefined ? REASON_NAMES[c] : ('код ' + c); }

function diagRow(parent, key, value) {
  const row = document.createElement('div');
  row.className = 'kv';
  const k = document.createElement('b'); k.textContent = key;
  const v = document.createElement('span'); v.textContent = value;
  row.appendChild(k); row.appendChild(v);
  parent.appendChild(row);
}

const ALARM_NAMES = ['перегрев коврика', 'перегрузка по току', 'SSR залип', 'обрыв нагрузки', 'отказ датчика воздуха', 'отказ датчика коврика'];
const SENSOR_NAMES = ['коврик', 'воздух', 'улица'];
const OTA_STATES = { '-1': 'нет данных (откат не включён)', 0: 'новая', 1: 'ожидает подтверждения', 2: 'подтверждена', 3: 'неисправна', 4: 'прервана' };

function alarmText(mask) {
  const out = [];
  ALARM_NAMES.forEach(function(n, i) { if (mask & (1 << i)) out.push(n); });
  return out.length ? out.join(', ') : 'нет';
}
function t10(v) { return v === -1270 ? '--' : (v / 10).toFixed(1); }
function kb(v) { return (v / 1024).toFixed(0); }

function eventText(e) {
  const a = e[3], b = e[4];
  switch (e[2]) {
    case 1: return 'Запуск: ' + reasonName(a);
    case 2: return 'Авария возникла: ' + alarmText(a);
    case 3: return 'Авария снята: ' + alarmText(a);
    case 4: return 'Wi-Fi: разрыв, причина ' + (WIFI_REASONS[a] ? WIFI_REASONS[a] + ' (' + a + ')' : 'код ' + a) + (b ? ', был подключён ' + fmtDur(b) : '');
    case 5: return 'Wi-Fi: подключено, сигнал ' + a + ' дБм (подключение №' + b + ')';
    case 6: return 'Время синхронизировано (NTP)';
    case 7: return 'Проверка версии OTA: ' + (a === 1 ? 'успешно' : 'не удалась') + ', ' + b + ' мс';
    case 8: return 'Датчик «' + SENSOR_NAMES[a] + '»: ' + (b === 85 ? 'ответил 85 °C (датчик перезагрузился по питанию)' : 'нет ответа');
    case 9: return 'Датчик «' + SENSOR_NAMES[a] + '»: переинициализация №' + b;
    case 10: return 'Задержка цикла ' + b + ' мс на этапе «' + (STAGE_NAMES[a] !== undefined ? STAGE_NAMES[a] : a) + '»';
    case 11: return 'Память: новый минимум ' + kb(a) + ' КБ (свободно ' + kb(b) + ' КБ)';
    case 12: return 'Аварийный таймерный режим ' + (a ? 'включён' : 'выключен');
    case 13: return 'ЭМР ' + (a ? 'замкнут' : 'разомкнут');
    case 14: return 'Энкодер: ' + a + ' невозможных переходов за 10 с (помехи?), всего ' + b;
    default: return 'Событие ' + e[2] + ' (' + a + ', ' + b + ')';
  }
}

function sect(box, title) {
  const h = document.createElement('h3');
  h.textContent = title;
  box.appendChild(h);
}

function renderDiag(d) {
  const box = document.getElementById('diagBody');
  while (box.firstChild) box.removeChild(box.firstChild);

  sect(box, 'Состояние сейчас');
  diagRow(box, 'Температуры: улица / воздух / коврик', t10(d.st[0]) + ' / ' + t10(d.st[1]) + ' / ' + t10(d.st[2]) + ' °C');
  diagRow(box, 'Уставка', d.st[3] + ' °C');
  diagRow(box, 'Нагрев', 'SSR ' + (d.st[5] ? 'включён' : 'выключен') + ', ЭМР ' + (d.st[6] ? 'замкнут' : 'разомкнут') + (d.st[8] ? ', аварийный таймерный режим' : ''));
  diagRow(box, 'Ток нагревателя', d.st[4] + ' мА');
  diagRow(box, 'Аварии', alarmText(d.st[7]));

  sect(box, 'Работа контроллера');
  diagRow(box, 'Работает без перезагрузки', fmtDur(d.up) + (d.now ? ' (с ' + fmtTime(d.now - d.up) + ')' : ''));
  diagRow(box, 'Причина последнего сброса', reasonName(d.rr));
  diagRow(box, 'Итераций цикла в секунду', d.sy[6]);
  diagRow(box, 'Самая долгая итерация цикла', d.ml + ' мс');
  const slow = [];
  d.sm.forEach(function(ms, i) { if (ms >= 50 && i !== 10) slow.push(STAGE_NAMES[i] + ': ' + ms + ' мс'); });
  diagRow(box, 'Дольше всего по этапам', slow.length ? slow.join('; ') : 'все этапы быстрее 50 мс');
  diagRow(box, 'Свободный стек цикла (минимум)', d.sy[5] ? d.sy[5] + ' байт' : 'ещё не измерен');
  diagRow(box, 'Включений SSR / переключений ЭМР', d.sy[7] + ' / ' + d.sy[8]);
  diagRow(box, 'Проверка версии (в фоне)', d.oc[0] ? (d.oc[0] + ' раз; последняя ' + d.oc[1] + ' мс, ' + (d.oc[2] === 1 ? 'успешно' : 'не удалась')) : 'ещё не выполнялась');
  diagRow(box, 'Состояние прошивки (откат OTA)', OTA_STATES[d.sy[9]] !== undefined ? OTA_STATES[d.sy[9]] : ('код ' + d.sy[9]));

  sect(box, 'Память и Wi-Fi');
  diagRow(box, 'Память: свободно / минимум / крупный блок',
    kb(d.heap[0]) + ' / ' + kb(d.heap[1]) + ' / ' + kb(d.heap[2]) + ' КБ');
  diagRow(box, 'Wi-Fi', d.rssi ? (d.rssi + ' дБм, канал ' + d.sy[10] + ', ' + d.ip + ', подключён ' + fmtDur(d.sy[11])) : 'нет связи');
  diagRow(box, 'Подключений / разрывов', d.wc + ' / ' + d.wd);
  if (d.wd > 0) diagRow(box, 'Причина последнего разрыва', WIFI_REASONS[d.wr] ? (WIFI_REASONS[d.wr] + ' (' + d.wr + ')') : ('код ' + d.wr));

  sect(box, 'Датчики температуры');
  d.sn.forEach(function(r, i) {
    // r: [найден, ошибок подряд, попыток переинициализации, обрывов связи, ответов 85 °C, переинициализаций всего]
    diagRow(box, SENSOR_NAMES[i], (r[0] ? 'найден' : 'НЕ найден') + '; ошибок подряд ' + r[1] + '; обрывов ' + r[3] + '; ответов 85 °C ' + r[4] + '; переинициализаций ' + r[5]);
  });

  sect(box, 'Помехи и ток');
  diagRow(box, 'Энкодер: прерываний / невозможных переходов', d.en[0] + ' / ' + d.en[1]);
  diagRow(box, 'Датчик тока: шумовой уровень', d.cs[1] ? (d.cs[0] + ' ед. АЦП') : 'не откалиброван');

  sect(box, 'Система');
  diagRow(box, 'Чип', d.chip + ', ревизия ' + d.sy[1] + ', ' + d.sy[0] + ' МГц, flash ' + d.sy[2] + ' МБ');
  diagRow(box, 'Прошивка в памяти', d.sy[3] + ' КБ (свободно ' + d.sy[4] + ' КБ)');
  diagRow(box, 'SDK', d.sdk);

  sect(box, 'События (сверху - самые свежие)');
  if (d.ev.length === 0) {
    diagRow(box, 'События', 'пока нет');
  } else {
    d.ev.forEach(function(e) {
      const ev = document.createElement('div');
      ev.className = 'ev';
      const when = e[1] ? fmtTime(e[1]) : ('через ' + fmtDur(e[0]) + ' после запуска');
      const head = document.createElement('strong');
      head.textContent = when;
      ev.appendChild(head);
      const body = document.createElement('div');
      body.textContent = eventText(e);
      ev.appendChild(body);
      box.appendChild(ev);
    });
  }
  if (d.sy[12] > 0) diagRow(box, 'Подавлено повторов событий', d.sy[12]);

  sect(box, 'Сбросы');
  const parts = [];
  d.cnt.forEach(function(n, i) { if (n > 0) parts.push(reasonName(i) + ': ' + n); });
  diagRow(box, 'Сбросов по типам', parts.length ? parts.join('; ') : 'нет данных');

  const h = document.createElement('h3');
  h.textContent = 'Журнал перезагрузок (сверху - самая свежая)';
  box.appendChild(h);
  if (d.log.length === 0) {
    diagRow(box, 'Журнал', 'пуст');
    return;
  }
  d.log.forEach(function(r, i) {
    // r: [причина, этап, источник, эпоха, аптайм, минПамять, максЦикл, воздух, флаги, аварии]
    const ev = document.createElement('div');
    ev.className = 'ev';
    const head = document.createElement('strong');
    head.textContent = (i + 1) + '. ' + reasonName(r[0]) + (i === 0 ? ' (этим сбросом начался текущий запуск)' : '');
    ev.appendChild(head);
    const body = document.createElement('div');
    if (r[2] === 0) {
      body.textContent = 'данных о предыдущем запуске нет';
    } else {
      const lines = [];
      lines.push('предыдущий запуск проработал ' + fmtDur(r[4]));
      lines.push(r[3] ? ('последняя активность: ' + fmtTime(r[3]) + (r[2] === 2 ? ' (+/- 10 мин)' : '')) : 'время последней активности неизвестно (не было NTP)');
      lines.push('место в программе: ' + (STAGE_NAMES[r[1]] !== undefined ? STAGE_NAMES[r[1]] : r[1]));
      lines.push('нагрев: SSR ' + ((r[8] & 1) ? 'включён' : 'выключен') + ', ЭМР ' + ((r[8] & 2) ? 'замкнут' : 'разомкнут'));
      lines.push('воздух: ' + (r[7] === -1270 ? 'нет данных' : (r[7] / 10).toFixed(1) + ' °C'));
      lines.push('минимум памяти: ' + kb(r[5]) + ' КБ, самая долгая итерация: ' + r[6] + ' мс');
      if (r[9]) lines.push('аварии: ' + alarmText(r[9]));
      body.textContent = lines.join('; ');
    }
    ev.appendChild(body);
    box.appendChild(ev);
  });
}

function updateDiag() {
  fetch('/api/diag').then(function(r) {
    if (!r.ok) throw new Error('diag HTTP ' + r.status);
    return r.json();
  }).then(renderDiag).catch(function() { /* повторим на следующем тике */ });
}

setInterval(updateDiag, 30000);
updateDiag();
</script>
</body>
</html>
)HTMLPAGE";

void handleDiagPage() {
  server.send_P(200, "text/html", DIAG_HTML);
}

void handleGraphs() {
  server.send_P(200, "text/html", GRAPHS_HTML);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/graphs", HTTP_GET, handleGraphs);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/history", HTTP_GET, handleApiHistory);
  server.on("/diag", HTTP_GET, handleDiagPage);
  server.on("/api/diag", HTTP_GET, handleApiDiag);
  server.on("/api/setpoint", HTTP_POST, handleApiSetpoint);
  server.on("/api/ota/confirm", HTTP_POST, handleApiOtaConfirm);
  server.on("/api/ota/decline", HTTP_POST, handleApiOtaDecline);
  // ВАЖНО: server.begin() здесь НЕ вызывается - только регистрация
  // маршрутов. Сам сервер поднимается позже, только когда Wi-Fi реально
  // подключится (см. updateWifiConnection()) - без сети поднимать его
  // незачем, всё равно никто не достучится.
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  // --- пины реле - САМЫМ первым делом, до любой другой инициализации ---
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, SSR_INACTIVE_LEVEL);

  pinMode(PIN_EMR, OUTPUT);
  digitalWrite(PIN_EMR, EMR_DEENERGIZED_LEVEL); // ЭМР разомкнут при старте -
                                                 // включится через EMR_STARTUP_DELAY_MS,
                                                 // см. updateEmr()/systemReady
  emrClosed = false;
  systemStartMs = millis();

  // диагностика: разобрать, чем закончился предыдущий запуск (сразу после
  // безопасных уровней на реле, до всего остального)
  diagBoot();
  lastEncoderActivityMs = millis(); // отсчёт автовыключения подсветки начинается с момента старта

  // --- экран и стартовая заставка с версией - сразу, до инициализации
  // датчиков (она занимает несколько секунд), чтобы экран не оставался
  // чёрным. Подсветку включаем только ПОСЛЕ отрисовки заставки (иначе на
  // долю секунды виден мусор/белый экран).
  pinMode(PIN_TFT_LED, OUTPUT);
  digitalWrite(PIN_TFT_LED, LOW);
  tft.init();
  tft.setRotation(2); // портретная ориентация, перевёрнутая (240 x 320)
  drawVersionSplashScreen();
  digitalWrite(PIN_TFT_LED, HIGH);

  // идентификатор текущего запуска - нужен странице графиков, чтобы понять,
  // что контроллер перезагружался и накопленную историю надо запросить заново
  historyBootId = esp_random() & 0x7FFFFFFF;

  // --- Wi-Fi и веб-сервер запускаем в начале - подключение идёт в фоне
  // (неблокирующе), параллельно с остальной инициализацией ниже ---
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2)
  WiFi.onEvent(diagOnWifiEvent); // счётчики подключений/разрывов Wi-Fi
#endif
  startWifiConnection();
  setupWebServer();

  // --- энкодер ---
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  // фиксируем начальное состояние до навешивания прерываний
  encoderPrevState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

  // прерывания на ОБА пина - иначе часть переходов (изменение только B)
  // будет пропущена, и декодер потеряет синхронизацию
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  // --- АЦП датчика тока ---
  analogReadResolution(12); // 0..4095, ESP32 по умолчанию и так 12 бит, но зафиксируем явно
  analogSetPinAttenuation(PIN_CURRENT, ADC_11db); // расширенный диапазон входа для АЦП

  // измеряем собственный шум датчика тока СРАЗУ, пока SSR гарантированно
  // выключен (задержка первого включения SSR - см. SSR_STARTUP_DELAY_MS) -
  // это даёт корректный порог "SSR залип" независимо от того, откалиброваны
  // ли ещё CURRENT_CALIB_* константы
  measureCurrentNoiseBaseline();

  // --- EEPROM ---
  EEPROM.begin(EEPROM_SIZE);
  loadSetpointFromEeprom();

  // --- датчики температуры (блокирует на несколько секунд - заставка с
  // версией уже на экране) ---
  initSensors();

  // Отсчёт 5 секунд показа версии (см. VERSION_SPLASH_DURATION_MS) начинаем
  // только теперь, когда блокирующая инициализация закончилась - иначе часть
  // времени заставки "съедалась" бы инициализацией датчиков. Обычный экран с
  // показаниями появится сам через updateDisplay() в loop().
  versionSplashStartMs = millis();

  // первый цикл опроса запускается сразу (не дожидаясь TEMP_POLL_INTERVAL_MS),
  // но сама последовательность неблокирующая и завершится по ходу первых
  // итераций loop() (см. updateTempPollStateMachine())
  startTempPollCycle();
  updateDisplay();

  // watchdog включаем в самом конце setup() - вся блокирующая инициализация
  // позади, дальше loop() обязан "отмечаться" не реже WDT_TIMEOUT_S секунд
  wdtStart();

  diagBusyClear(); // запуск завершён - снимаем метку "завис при старте"
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
unsigned long lastTempPollMs = 0;
unsigned long lastCurrentDisplayMs = 0;

// Откат неудачного OTA-обновления: новая прошивка помечается как рабочая
// НЕ сразу при старте, а только после того, как она прожила
// OTA_VALIDATE_AFTER_MS и полностью отработала хотя бы один цикл опроса
// датчиков. Если до этого момента устройство упадёт/зависнет (watchdog) и
// перезагрузится, загрузчик вернётся на предыдущую прошивку - но только если
// в нём включён откат (в стандартной сборке ядра Arduino ESP32 это зависит от
// версии ядра; без поддержки вызов ниже просто ничего не делает).
const unsigned long OTA_VALIDATE_AFTER_MS = 60UL * 1000UL;
bool otaImageMarkedValid = false;

// Говорит ядру Arduino ESP32 НЕ помечать прошивку рабочей автоматически
// перед setup() (если откат в сборке включён) - решение принимаем сами, см.
// выше. Если ядро этой функции не знает - она просто не вызывается.
extern "C" bool verifyRollbackLater() {
  return true;
}

void loop() {
  wdtFeed();
  diagTick();
  diagStage(STAGE_BUTTONS);

  if (!otaImageMarkedValid && tempPollCompletedOnce &&
      (millis() - systemStartMs >= OTA_VALIDATE_AFTER_MS)) {
    esp_ota_mark_app_valid_cancel_rollback();
    otaImageMarkedValid = true;
  }

  handleButton();
  handleEncoder();
  checkEncoderEditTimeout();
  checkWifiSetupConfirmTimeout();
  checkBacklightAutoOff();

  if (systemMode == MODE_WIFI_SETUP_ACTIVE) {
    // портал WiFiManager занят своими делами - наш собственный веб-сервер
    // в это время остановлен (см. startWifiSetupPortal()), поэтому
    // server.handleClient() здесь не вызываем
    diagStage(STAGE_WIFI);
    updateWifiSetupPortal();
  } else {
    diagStage(STAGE_WIFI);
    updateWifiConnection(); // неблокирующая проверка/переподключение Wi-Fi
    updateTimeSync();       // неблокирующая проверка/повтор синхронизации NTP
    updateOtaCheck();       // неблокирующий (по частоте) запуск проверки версии прошивки
    diagStage(STAGE_WEB);
    server.handleClient();  // обработка входящих HTTP-запросов, если они есть
  }

  // подтверждённое пользователем обновление прошивки - запускаем именно
  // здесь (а не прямо в HTTP-обработчике), чтобы сначала успеть отправить
  // ответ браузеру. Сама операция БЛОКИРУЕТ выполнение на время скачивания
  // и записи (см. подробности в performFirmwareUpdate()).
  if (otaConfirmedTriggered && !otaCheckTaskRunning) { // флаг подождёт, пока закончится фоновая проверка версии
    otaConfirmedTriggered = false;
    performFirmwareUpdate();
  }

  unsigned long now = millis();

  // задержка включения ЭМР после старта (см. EMR_STARTUP_DELAY_MS) -
  // проверяется один раз, флаг не сбрасывается обратно
  if (!systemReady && (now - systemStartMs >= EMR_STARTUP_DELAY_MS)) {
    systemReady = true;
  }

  // отдельная, более долгая задержка первого включения SSR (см. SSR_STARTUP_DELAY_MS)
  if (!ssrStartupDelayPassed && (now - systemStartMs >= SSR_STARTUP_DELAY_MS)) {
    ssrStartupDelayPassed = true;
  }

  if (now - lastTempPollMs >= TEMP_POLL_INTERVAL_MS && tempPollState == TPOLL_IDLE) {
    lastTempPollMs = now;
    startTempPollCycle();
  }
  diagStage(STAGE_SENSORS);
  updateTempPollStateMachine(); // продвигает неблокирующий опрос датчиков, если он идёт

  diagStage(STAGE_CURRENT);
  checkCurrent();      // сама регулирует частоту вызова внутри себя (быстро, для аварий)

  if (now - lastHistorySampleMs >= HISTORY_SAMPLE_INTERVAL_MS) {
    lastHistorySampleMs = now;
    recordHistorySample();
  }

  if (now - lastCurrentDisplayMs >= CURRENT_DISPLAY_INTERVAL_MS) {
    lastCurrentDisplayMs = now;
    currentAmpsForDisplay = currentAmpsHundredths; // снимок для показа раз в 5 секунд
  }

  diagStage(STAGE_THERMOSTAT);
  updateThermostat();
  updateEmr();

  // при выключенной подсветке экран не виден - пропускаем все обновления
  // отрисовки; "lastShown..." трекеры при этом не трогаются, так что когда
  // подсветка включится обратно, обычная логика "перерисовать только при
  // изменении" сама корректно досчитает всё, что накопилось за это время
  if (backlightOn) {
    diagStage(STAGE_DISPLAY);
    updateDisplay();
  }
  diagStage(STAGE_LOOP);
}
