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
    EMR  -> GPIO32  (HIGH = катушка запитана = цепь ЗАМКНУТА, штатный режим;
                      LOW  = авария/сброс питания = цепь РАЗОМКНУТА, fail-safe)
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
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <EEPROM.h>

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
const int EEPROM_SIZE = 8;
const int EEPROM_ADDR_MAGIC = 0;         // 1 байт - признак "данные валидны"
const int EEPROM_ADDR_SETPOINT = 1;      // 1 байт - уставка
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
const int8_t ENCODER_TRANSITION_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

volatile uint8_t encoderPrevState = 0;
volatile int32_t encoderRawDelta = 0;

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t currState = (a << 1) | b;
  uint8_t index = (encoderPrevState << 2) | currState;
  encoderRawDelta += ENCODER_TRANSITION_TABLE[index];
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
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);
  EEPROM.write(EEPROM_ADDR_SETPOINT, (uint8_t)value);
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
  int32_t t = (int32_t)raw * 5;
  t = t / 64; // 5/64 == 10/128
  return (int16_t)t;
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
    if (raw == DEVICE_DISCONNECTED_RAW) {
      tempMatError = true; // старое значение tempMat НЕ трогаем
      if (matErrorStreak < 0xFFFF) matErrorStreak++;
    } else {
      tempMat = rawToTenths(raw);
      tempMatError = false;
      matErrorStreak = 0;
      matReinitAttempts = 0; // связь восстановлена - счётчик попыток тоже сбрасываем
    }
  } else {
    tempMatError = true; // датчик вообще не был найден при старте
    if (matErrorStreak < 0xFFFF) matErrorStreak++;
  }
  // для коврика ЕСТЬ отдельная авария (см. ALARM_MAT_SENSOR_FAIL) - но она
  // не останавливает нагрев сама по себе, только предупреждает
  bool matReinitExhausted = handleSensorErrorStreak(matErrorStreak, matReinitAttempts, dsMat, matFound, addrMat);
  if (matReinitExhausted) {
    alarmFlags |= ALARM_MAT_SENSOR_FAIL;
  }

  if (airFound) {
    int16_t raw = dsAir.getTemp(addrAir);
    if (raw == DEVICE_DISCONNECTED_RAW) {
      tempAirError = true;
      if (airErrorStreak < 0xFFFF) airErrorStreak++;
    } else {
      tempAir = rawToTenths(raw);
      tempAirError = false;
      airErrorStreak = 0; // успешное чтение сбрасывает счётчик подряд идущих ошибок
      airReinitAttempts = 0;
    }
  } else {
    tempAirError = true;
    if (airErrorStreak < 0xFFFF) airErrorStreak++;
  }
  // для опорного датчика переинициализация - это ПОСЛЕДНИЙ ШАНС перед
  // аварией: авария поднимается только когда попытки уже исчерпаны
  bool airReinitExhausted = handleSensorErrorStreak(airErrorStreak, airReinitAttempts, dsAir, airFound, addrAir);
  if (airReinitExhausted) {
    alarmFlags |= ALARM_AIR_SENSOR_FAIL;
  }

  if (outFound) {
    int16_t raw = dsOut.getTemp(addrOut);
    if (raw == DEVICE_DISCONNECTED_RAW) {
      tempOutError = true;
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

  checkMatOverheat();
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

// ---------------------------------------------------------------------------
// АВАРИЙНАЯ ЗАЩИТА ПО ТЕМПЕРАТУРЕ КОВРИКА
// ---------------------------------------------------------------------------
void checkMatOverheat() {
  if (tempMat != -1270 && tempMat > MAT_TEMP_ALARM_TENTHS) {
    alarmFlags |= ALARM_MAT_OVERHEAT;
  }
}

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
      openLoadCounter++;
      if (openLoadCounter >= CURRENT_FAULT_CONFIRM_COUNT) {
        alarmFlags |= ALARM_OPEN_LOAD; // не разрываем ЭМР - не опасно, просто сигнал
      }
    } else {
      openLoadCounter = 0;
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

void drawWifiStatus() {
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

unsigned long lastHistorySampleMs = 0;

void recordHistorySample() {
  historyOut[historyWriteIndex] = tempOut;
  historyAir[historyWriteIndex] = tempAir;
  historyMat[historyWriteIndex] = tempMat;
  historyCurrentMa[historyWriteIndex] = (int16_t)((int32_t)currentAmpsForDisplay * 10); // сотые А -> мА

  historyWriteIndex = (historyWriteIndex + 1) % HISTORY_SIZE;
  if (historyFilledCount < HISTORY_SIZE) historyFilledCount++;
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
</style>
</head>
<body>
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

setInterval(updateStatus, 3000);
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
  json += "\"alarms\":" + alarmsJson;
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

// Добавляет один массив истории в JSON-строку, разворачивая кольцевой
// буфер в хронологический порядок (от самой старой точки к новой).
void appendHistoryArray(String &json, const int16_t* data, uint16_t count, uint16_t startIndex) {
  json += "[";
  for (uint16_t i = 0; i < count; i++) {
    uint16_t idx = (startIndex + i) % HISTORY_SIZE;
    if (i > 0) json += ",";
    json += String(data[idx]);
  }
  json += "]";
}

void handleApiHistory() {
  uint16_t count = historyFilledCount;
  // индекс самой старой точки в кольцевом буфере
  uint16_t startIndex = (historyWriteIndex + HISTORY_SIZE - count) % HISTORY_SIZE;

  String json;
  json.reserve(40000); // грубая оценка максимального размера с запасом - вся история за 24ч
  json += "{";
  json += "\"intervalSec\":" + String(HISTORY_SAMPLE_INTERVAL_MS / 1000) + ",";
  json += "\"maxCount\":" + String(HISTORY_SIZE) + ",";
  json += "\"timeSynced\":" + String(timeSynced ? "true" : "false") + ",";
  json += "\"nowEpoch\":" + String((unsigned long)time(nullptr)) + ",";
  json += "\"count\":" + String(count) + ",";
  json += "\"out\":";       appendHistoryArray(json, historyOut, count, startIndex);       json += ",";
  json += "\"air\":";       appendHistoryArray(json, historyAir, count, startIndex);       json += ",";
  json += "\"mat\":";       appendHistoryArray(json, historyMat, count, startIndex);       json += ",";
  json += "\"currentMa\":"; appendHistoryArray(json, historyCurrentMa, count, startIndex);
  json += "}";

  server.send(200, "application/json", json);
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
  .info { font-size:12px; color:#777; text-align:center; margin-top:8px; }
  button { width:100%; padding:16px; font-size:19px; background:#444; color:#fff; border:none; border-radius:10px; margin-top:4px; }
  button:active { background:#333; }
</style>
</head>
<body>
  <div class="card">
    <div class="title">Будка</div>
    <h2 class="subtitle">Температуры, &deg;C (24 часа)</h2>
    <div class="legend">
      <span><span class="dot" style="background:#4da6ff"></span>Снаружи</span>
      <span><span class="dot" style="background:#fff"></span>Внутри</span>
      <span><span class="dot" style="background:#ff5c5c"></span>Внизу</span>
    </div>
    <canvas id="tempChart" width="600" height="220"></canvas>
  </div>
  <div class="card">
    <h2 class="subtitle">Ток нагревателя, мА (24 часа)</h2>
    <div class="legend">
      <span><span class="dot" style="background:#ffa64d"></span>Ток</span>
    </div>
    <canvas id="curChart" width="600" height="220"></canvas>
    <div class="info" id="pointsInfo">...</div>
  </div>
  <div class="card">
    <button onclick="location.href='/'">Параметры</button>
  </div>
<script>
function drawChart(canvas, seriesList, maxSlots, forcePositive, timeSynced, nowEpoch) {
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
  const windowMinutes = Math.max(1, Math.min(dataCount, maxSlots));

  // --- шаг сетки по времени подбирается под размер окна ---
  let gridStepMin;
  if (windowMinutes <= 60) gridStepMin = 10;
  else if (windowMinutes <= 120) gridStepMin = 15;
  else if (windowMinutes <= 240) gridStepMin = 30;
  else gridStepMin = 60;

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

  // --- вертикальная сетка по времени: считается от правого края (сейчас)
  // назад с шагом gridStepMin - подписываем не каждую линию, а примерно
  // раз в 5-6 линий, чтобы не было тесно ---
  const totalLines = Math.floor(windowMinutes / gridStepMin);
  const labelEvery = Math.max(1, Math.round(totalLines / 6));

  ctx.strokeStyle = '#2a2a2a';
  ctx.fillStyle = '#2ecc71';
  ctx.font = '12px sans-serif';
  for (let lineNum = 0; lineNum <= totalLines; lineNum++) {
    const m = lineNum * gridStepMin;
    const x = marginLeft + plotW * (1 - m / windowMinutes);
    ctx.beginPath();
    ctx.moveTo(x, marginTop);
    ctx.lineTo(x, marginTop + plotH);
    ctx.stroke();

    if (lineNum % labelEvery === 0) {
      let label;
      if (timeSynced && nowEpoch > 0) {
        // реальное время получено по NTP - показываем часы:минуты
        const ts = new Date((nowEpoch - m * 60) * 1000);
        const hh = ('0' + ts.getHours()).slice(-2);
        const mm = ('0' + ts.getMinutes()).slice(-2);
        label = hh + ':' + mm;
      } else {
        // NTP недоступен (нет интернета у Wi-Fi сети) - относительное время, как раньше
        if (m === 0) label = 'now';
        else if (m < 60) label = '-' + m + 'm';
        else label = '-' + (m % 60 === 0 ? (m / 60) : (m / 60).toFixed(1)) + 'h';
      }
      ctx.fillText(label, Math.max(marginLeft, x - 12), h - 3);
    }
  }

  // --- сами линии данных: самая свежая точка всегда у правого края,
  // окно = windowMinutes (см. выше) ---
  seriesList.forEach(function(s) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    const n = s.data.length;
    for (let i = 0; i < n; i++) {
      const v = s.data[i];
      const distFromNewest = (n - 1 - i); // 0 = самая свежая точка
      const x = marginLeft + plotW * (1 - distFromNewest / windowMinutes);
      if (v === null) { started = false; continue; }
      const y = marginTop + plotH * (1 - (v - minV) / (maxV - minV));
      if (!started) { ctx.moveTo(x, y); started = true; }
      else { ctx.lineTo(x, y); }
    }
    ctx.stroke();
  });
}

function updateGraphs() {
  fetch('/api/history').then(function(r){ return r.json(); }).then(function(d){
    function conv10(arr) {
      return arr.map(function(v) { return v === -1270 ? null : v / 10; });
    }

    drawChart(document.getElementById('tempChart'), [
      { data: conv10(d.out), color: '#4da6ff' },
      { data: conv10(d.air), color: '#fff' },
      { data: conv10(d.mat), color: '#ff5c5c' }
    ], d.maxCount, false, d.timeSynced, d.nowEpoch);
    drawChart(document.getElementById('curChart'), [
      { data: d.currentMa, color: '#ffa64d' }
    ], d.maxCount, true, d.timeSynced, d.nowEpoch);

    let hours = (d.count * d.intervalSec / 3600).toFixed(1);
    let timeStatus = d.timeSynced ? 'время: NTP' : 'время: относительное (нет NTP)';
    document.getElementById('pointsInfo').textContent = d.count + ' точек (~' + hours + ' ч), интервал ' + d.intervalSec + ' с, ' + timeStatus;
  });
}

setInterval(updateGraphs, 20000);
updateGraphs();
</script>
</body>
</html>
)HTMLPAGE";

void handleGraphs() {
  server.send_P(200, "text/html", GRAPHS_HTML);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/graphs", HTTP_GET, handleGraphs);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/history", HTTP_GET, handleApiHistory);
  server.on("/api/setpoint", HTTP_POST, handleApiSetpoint);
  // ВАЖНО: server.begin() здесь НЕ вызывается - только регистрация
  // маршрутов. Сам сервер поднимается позже, только когда Wi-Fi реально
  // подключится (см. updateWifiConnection()) - без сети поднимать его
  // незачем, всё равно никто не достучится.
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  lastEncoderActivityMs = millis(); // отсчёт автовыключения подсветки начинается с момента старта

  // --- Wi-Fi и веб-сервер запускаем сразу в начале - подключение идёт в
  // фоне (неблокирующе), параллельно с остальной инициализацией ниже ---
  startWifiConnection();
  setupWebServer();

  // --- пины реле ---
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, SSR_INACTIVE_LEVEL);

  pinMode(PIN_EMR, OUTPUT);
  digitalWrite(PIN_EMR, EMR_DEENERGIZED_LEVEL); // ЭМР разомкнут при старте -
                                                 // включится через EMR_STARTUP_DELAY_MS,
                                                 // см. updateEmr()/systemReady
  emrClosed = false;
  systemStartMs = millis();

  // --- подсветка ---
  pinMode(PIN_TFT_LED, OUTPUT);
  digitalWrite(PIN_TFT_LED, HIGH);

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

  // --- датчики температуры ---
  initSensors();

  // --- экран ---
  tft.init();
  tft.setRotation(2); // портретная ориентация, перевёрнутая (240 x 320)
  drawStaticLabels();

  // первый цикл опроса запускается сразу (не дожидаясь TEMP_POLL_INTERVAL_MS),
  // но сама последовательность неблокирующая и завершится по ходу первых
  // итераций loop() (см. updateTempPollStateMachine())
  startTempPollCycle();
  updateDisplay();
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
unsigned long lastTempPollMs = 0;
unsigned long lastCurrentDisplayMs = 0;

void loop() {
  handleButton();
  handleEncoder();
  checkEncoderEditTimeout();
  checkWifiSetupConfirmTimeout();
  checkBacklightAutoOff();

  if (systemMode == MODE_WIFI_SETUP_ACTIVE) {
    // портал WiFiManager занят своими делами - наш собственный веб-сервер
    // в это время остановлен (см. startWifiSetupPortal()), поэтому
    // server.handleClient() здесь не вызываем
    updateWifiSetupPortal();
  } else {
    updateWifiConnection(); // неблокирующая проверка/переподключение Wi-Fi
    updateTimeSync();       // неблокирующая проверка/повтор синхронизации NTP
    server.handleClient();  // обработка входящих HTTP-запросов, если они есть
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
  updateTempPollStateMachine(); // продвигает неблокирующий опрос датчиков, если он идёт

  checkCurrent();      // сама регулирует частоту вызова внутри себя (быстро, для аварий)

  if (now - lastHistorySampleMs >= HISTORY_SAMPLE_INTERVAL_MS) {
    lastHistorySampleMs = now;
    recordHistorySample();
  }

  if (now - lastCurrentDisplayMs >= CURRENT_DISPLAY_INTERVAL_MS) {
    lastCurrentDisplayMs = now;
    currentAmpsForDisplay = currentAmpsHundredths; // снимок для показа раз в 5 секунд
  }

  updateThermostat();
  updateEmr();

  // при выключенной подсветке экран не виден - пропускаем все обновления
  // отрисовки; "lastShown..." трекеры при этом не трогаются, так что когда
  // подсветка включится обратно, обычная логика "перерисовать только при
  // изменении" сама корректно досчитает всё, что накопилось за это время
  if (backlightOn) {
    updateDisplay();
  }
}
