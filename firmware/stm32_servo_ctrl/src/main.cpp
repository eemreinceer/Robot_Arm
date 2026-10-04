#include <Arduino.h>

extern "C" {
#include "protocol.h"
}

#include <cstring>
#include <cstdio>

#define LINE_SIZE 80U
#define COMMAND_WATCHDOG_MS 1000U
#define FW_VERSION "0.6.0-uart-pb6"

/* Protokol tasiyicisi: USART1 (PA9 TX / PA10 RX), 115200 8N1.
 *
 * NOT (2026-07-14, donanimda dogrulandi): bu karttaki GD32F103 klonunda USB
 * CDC kullanilamiyor. Cip enumerate oluyor ve EP0/kontrol transferleri
 * calisiyor (DTR goruluyor), ancak HER IKI bulk endpoint de sonsuz NAK
 * veriyor - usbmon ile teyit edildi: host "V\n" (560a) gonderiyor, URB'ler
 * hic tamamlanmiyor, port kapaninca -2 (ENOENT) ile iptal ediliyor.
 * Bu yuzden tasima UART; USB stack'i tamamen devre disi.
 *
 * Bu hat protokolden BASKA hicbir sey basmamali: ROS tarafi (
 * stm32_system_interface.cpp) yaniti tam olarak "OK" diye karsilastiriyor,
 * araya kacacak tek bir debug satiri handshake'i bozar. Teshis LED'de.
 */
static HardwareSerial Link(PA10, PA9);

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4; /* kanal 3 = TIM4_CH1/PB6 */
static uint32_t last_position_ms;
static bool pwm_enabled;
static bool watchdog_reported;

/* Kesmeleri KAPATMA: cip sessizce olmemeli, LED'den kod blink etsin.
 * (Onceki firmware burada noInterrupts() yapiyordu; ayrica SysTick_Handler
 * hic tanimli olmadigi icin Default_Handler -> Infinite_Loop'a dusuyordu ve
 * cip her boot'ta ~1ms sonra doniyordu. Arduino core artik SysTick'i sagliyor.) */
static void fatal_error(uint8_t code) {
  pinMode(PC13, OUTPUT);
  for (;;) {
    for (uint8_t i = 0; i < code; i++) {
      digitalWrite(PC13, LOW);
      delay(150);
      digitalWrite(PC13, HIGH);
      delay(150);
    }
    delay(1200);
  }
}

static void link_write(const char *text) {
  Link.write(reinterpret_cast<const uint8_t *>(text), std::strlen(text));
}

/* Kanal 3 = PB6/TIM4_CH1. Onceki iki deneme OLCULDU ve ikisi de olu cikti:
 * PA2/TIM2_CH3 ve PB0/TIM3_CH3. Ayni kayitlarda TIM2_CH1 (PA0, 500.1 us),
 * TIM2_CH2 (PA1, 1500.4 us) ve TIM3_CH1 (PA6, 1000.3 us) mikrosaniye
 * hassasiyetinde dogru basiyordu -- yani iki timer da doniyor, sadece
 * ucuncu kanallari olu. Logic analyzer, kanali PA1'de dogrulanmis halde,
 * PB0'da 200k ornekte SIFIR kenar gordu.
 *
 * Kod yolu bu arizayi ACIKLAMIYOR: parser, apply_positions, HAL makrosu,
 * HAL_TIM_PWM_Start ve TIM_CCxChannelCmd okundu, hepsi kanal-agnostik.
 * Ustelik PA2 gun icinde bir ara CALISTI (uc tur temiz supurme), ki bu da
 * "CHANNEL_3 kodu bastan bozuk" aciklamasini celiyor. Kok neden BILINMIYOR.
 *
 * TIM4 secildi cunku tamamen bosta ve CH1 -- iki ayri timer'da calistigi
 * olculmus olan kanal numarasi. Bu bir kacinma, cozum degil; asil ayrim
 * icin 'R' teshis komutu eklendi (CCER/CCR dokumu).
 */
static void pwm_stop_all() {
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  pwm_enabled = false;
}

static void pwm_start_all() {
  if (pwm_enabled) {
    return;
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
    fatal_error(5);
  }
  pwm_enabled = true;
}

static void apply_positions(const uint16_t pulse_us[SERVO_CHANNEL_COUNT]) {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us[0]);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us[1]);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us[2]);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_us[3]);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us[4]);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us[5]);
  pwm_start_all();
  last_position_ms = millis();
  watchdog_reported = false;
}

/* 'R' -> register dokumu. Teshis icin: servo davranisindan geriye tahmin
 * yurutmek yerine cipe kendi CCER/CCR degerlerini sordurur. ROS tarafi bu
 * komutu hic gondermez; yaniti tek satir, protokolun geri kalanina dokunmaz. */
static void dump_registers() {
  char buffer[120];
  snprintf(buffer, sizeof(buffer),
           "T2 CCER=%04lX CCR1=%lu CCR2=%lu CCR4=%lu\n",
           (unsigned long)TIM2->CCER, (unsigned long)TIM2->CCR1,
           (unsigned long)TIM2->CCR2, (unsigned long)TIM2->CCR4);
  link_write(buffer);
  snprintf(buffer, sizeof(buffer),
           "T3 CCER=%04lX CCR1=%lu CCR2=%lu CCR3=%lu\n",
           (unsigned long)TIM3->CCER, (unsigned long)TIM3->CCR1,
           (unsigned long)TIM3->CCR2, (unsigned long)TIM3->CCR3);
  link_write(buffer);
  snprintf(buffer, sizeof(buffer),
           "T4 CCER=%04lX CCR1=%lu CR1=%04lX ARR=%lu PSC=%lu EN=%d\n",
           (unsigned long)TIM4->CCER, (unsigned long)TIM4->CCR1,
           (unsigned long)TIM4->CR1, (unsigned long)TIM4->ARR,
           (unsigned long)TIM4->PSC, pwm_enabled ? 1 : 0);
  link_write(buffer);
}

static void handle_line(const char *line) {
  if (std::strcmp(line, "R") == 0) {
    dump_registers();
    return;
  }

  protocol_command_t command;
  const protocol_result_t result = protocol_parse_line(line, &command);
  if (result != PROTOCOL_OK) {
    link_write(result == PROTOCOL_ERROR_CHANNEL_COUNT ? "E2\n" : "E1\n");
    return;
  }
  switch (command.type) {
    case PROTOCOL_COMMAND_POSITION:
      apply_positions(command.pulse_us);
      link_write("OK\n");
      break;
    case PROTOCOL_COMMAND_VERSION:
      link_write("V1," FW_VERSION "\n");
      break;
    case PROTOCOL_COMMAND_SAFE_STOP:
      pwm_stop_all();
      link_write("OK\n");
      break;
    default:
      link_write("E1\n");
      break;
  }
}

static void process_rx() {
  static char line[LINE_SIZE];
  static size_t length;
  while (Link.available() > 0) {
    const int input = Link.read();
    if (input < 0) {
      break;
    }
    const char value = (char)input;
    if (value == '\r') {
      continue;
    }
    if (value == '\n') {
      line[length] = '\0';
      handle_line(line);
      length = 0U;
    } else if (length + 1U < sizeof(line)) {
      line[length++] = value;
    } else {
      length = 0U;
      link_write("E1\n");
    }
  }
}

static void timer_base_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, uint8_t code) {
  timer->Instance = instance;
  timer->Init.Prescaler = (SystemCoreClock / 1000000U) - 1U; /* 1 MHz tick = 1 us */
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = 19999U; /* 20 ms = 50 Hz */
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(timer) != HAL_OK) {
    fatal_error(code);
  }
}

static void configure_channel(TIM_HandleTypeDef *timer, uint32_t channel, uint8_t code) {
  TIM_OC_InitTypeDef output = {};
  output.OCMode = TIM_OCMODE_PWM1;
  output.Pulse = 1500U;
  output.OCPolarity = TIM_OCPOLARITY_HIGH;
  output.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(timer, &output, channel) != HAL_OK) {
    fatal_error(code);
  }
}

static void pwm_init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {};
  /* PA2 KASITLI OLARAK YOK: kanal 3 artik PB6'da (bkz. pwm_stop_all notu). */
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 |
             GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  GPIO_InitTypeDef gpio_b = {};
  gpio_b.Pin = GPIO_PIN_6; /* PB6 = TIM4_CH1 = protokol kanali 3 */
  gpio_b.Mode = GPIO_MODE_AF_PP;
  gpio_b.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_b);
  timer_base_init(&htim2, TIM2, 1);
  configure_channel(&htim2, TIM_CHANNEL_1, 2);
  configure_channel(&htim2, TIM_CHANNEL_2, 2);
  configure_channel(&htim2, TIM_CHANNEL_4, 2);
  timer_base_init(&htim3, TIM3, 3);
  configure_channel(&htim3, TIM_CHANNEL_1, 4);
  configure_channel(&htim3, TIM_CHANNEL_2, 4);
  timer_base_init(&htim4, TIM4, 6);
  configure_channel(&htim4, TIM_CHANNEL_1, 7);
  pwm_stop_all();
}

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW); /* LED ON = setup icinde */
  Link.begin(115200);
  pwm_init();
  digitalWrite(PC13, HIGH); /* LED OFF = setup bitti */
}

void loop() {
  static uint32_t led_ms;
  static bool led;

  process_rx();

  if (pwm_enabled && !watchdog_reported &&
      (millis() - last_position_ms) >= COMMAND_WATCHDOG_MS) {
    /* Sozlesme: son PWM degerlerini KORU, timeout'u bir kez bildir. */
    watchdog_reported = true;
    link_write("E3\n");
  }

  /* 1 Hz blink = loop canli. Sabit yanik/sonuk = donmus. */
  if ((millis() - led_ms) >= 500U) {
    led_ms = millis();
    led = !led;
    digitalWrite(PC13, led ? LOW : HIGH);
  }
}
