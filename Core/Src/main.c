/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ====== receiver channel -> function (change here to swap sticks) ======
   MC6C has ONLY two layouts, no menu, no display.
   Tell them apart by hand: the stick whose up/down does NOT spring back
   to centre is the throttle stick, and the throttle is always CH3.
     RC_HAND_MODE 2 = 左手模式 : throttle on the LEFT stick  (MC6C factory default)
     RC_HAND_MODE 1 = 右手模式 : throttle on the RIGHT stick */
#define RC_HAND_MODE  2

#if RC_HAND_MODE == 2
  #define RC_CH_ROT   1     /* CH1 right stick L/R = rotate  -> PA8 (TIM1_CH1) */
  #define RC_CH_SERVO 2     /* CH2 right stick U/D = servo   -> PA9 (TIM1_CH2) */
  #define RC_CH_FWD   3     /* CH3 left  stick U/D = forward -> PB8 (TIM4_CH3) */
  #define RC_CH_LAT   4     /* CH4 left  stick L/R = strafe  -> PB9 (TIM4_CH4) */
#else
  #define RC_CH_ROT   1     /* CH1 right stick L/R = rotate  -> PA8 (TIM1_CH1) */
  #define RC_CH_SERVO 2     /* CH2 left  stick U/D = servo   -> PA9 (TIM1_CH2) */
  #define RC_CH_FWD   3     /* CH3 right stick U/D = forward -> PB8 (TIM4_CH3) */
  #define RC_CH_LAT   4     /* CH4 left  stick L/R = strafe  -> PB9 (TIM4_CH4) */
#endif

#define RC_TIMEOUT_MS 150   /* no pulse for this long -> fail-safe stop */
#define RC_DEADBAND   25    /* center dead band, us */

/* ---- mecanum straight-line trim ----
   Measured on this car:
     driving forward veers ~15 deg LEFT over 2-3 m, and driving backward
     veers right by the same amount -- the right-hand wheels run a little
     faster than the left ones;
     strafing either way picks up a forward component.
   Both are cancelled here, as a percentage of the stick command.
   0 = correction off.  Tune these two numbers against what the car does. */
/* all four wheels reach the SAME top speed, the difference is only in how
   much PWM each needs to break away -- the right side needs more, so at low
   stick the car gets pushed left.  A proportional trim would over-correct at
   full speed, so this is an offset that is full size at low stick and fades
   to nothing at full stick. */
#define MEC_KICK        14  /* % extra for the right wheels at low stick   */
#define MEC_STRAFE_TRIM  8  /* % : pull back while strafing                */

/* left stick: just off centre the command is 50%, then it rises in a straight
   line to 100% at full travel (see Stick_Curve).  On top of that the
   start-up kick gives a 75% burst from standstill.  Spin has its own curve,
   Spin_Curve. */
#define STICK_MIN_MOVE  50  /* % commanded just outside the dead band      */

/* ---- start-up kick ----
   The right-front wheel will not break away from standstill at 50% duty --
   it needs about 75% -- but once it is turning, 50% keeps it going.  So for
   a moment after the car leaves standstill every wheel is driven at 3/2 of
   the commanded speed (50% -> 75%) and then drops back.
   Caveat: this only masks a weak wheel.  If the wheel is really losing
   contact now and then, the kick will not save it. */
#define MOT_KICK_MS     150   /* how long the boost lasts                    */
#define MOT_KICK_NUM      3   /* boost = MOT_KICK_NUM / MOT_KICK_DEN = 1.5x  */
#define MOT_KICK_DEN      2

/* receiver fail-safe output: CH1/2/4 = 1500us, CH3 = 900us.
   900us must NOT be read as "reverse 100%" -> anything under this is
   treated as no signal, so a radio link loss stops the car. */
#define RC_MIN_US     960
#define RC_MAX_US     2100

/* ---- servo ----
   The servo ignores any pulse outside 500..2500us (2650 and 450 both dead),
   so those are the hard hardware ends.  1 deg ~= 9.87us.
   The servo was re-seated in its bracket with the shaft parked at 1500us,
   its own mechanical centre, so "straight ahead" now sits mid-travel and
   there is the same 90 deg of room on each side:
     stick fully DOWN -> 1500 + 888 = 2388us = 90 deg right
     stick CENTRE     -> 1500us             = straight ahead
     stick fully UP   -> 1500 - 888 =  612us = 90 deg left
   Both ends stay well inside 500..2500, nothing clips.                  */
/* straight ahead, measured on the car (already includes the trim needed to
   bring the arm square -- 1500us put it 5 deg right, so it is 1400us now).
   Both ends are pushed as far as the servo's 500..2500us limit allows, with
   a 20us margin so it never sits on the edge:
     left  (stick up)   -> 520us  =  880us of travel
     right (stick down) -> 2480us = 1080us of travel
   The stick is asymmetric only because "straight" is not exactly mid-travel;
   each side uses every microsecond the hardware will take.                */
#define SERVO_US_ZERO  1400     /* stick centred -> straight ahead        */
#define SERVO_US_MIN    572     /* stick fully up -> 80.5 deg left        */
/* right side measured: 2480us gave 105 deg, i.e. 10.29us per degree.
   the arm needs 80 deg, so stop at 1400 + 80*10.29 = 2223us.            */
#define SERVO_US_MAX   2223     /* stick fully down -> 80 deg right       */

/* output trim, us: + turns right, - turns left (already folded into
   SERVO_US_ZERO above; kept here for future fine adjustment) */
#define SERVO_TRIM_US        0

/* ---- installation mode ----
   Set to 1 to hold the servo shaft at 1500us, the servo's own mechanical
   centre, and ignore the stick.  Use it while re-seating the servo in its
   bracket: with the shaft parked exactly mid-travel the arm can be aligned
   to "straight ahead", which then leaves the same 90 deg of travel on each
   side.  Set back to 0 when the alignment is done.                        */
#define SERVO_INSTALL_MODE   0

/* wide dead zone right at the stick centre.  Once the stick is away from
   centre the arm is rock steady, but sitting at the middle it hunts: the
   transmitter's pulse wobbles enough to keep crossing the boundary.  The
   zone is deliberately generous -- 100us is 20% of the stick, which is still
   far more than the tiny nudge needed to start a turn.  The mapping
   below shrinks the usable stick travel to (500 - SERVO_CENTRE_DEAD) so
   leaving the zone starts the arm exactly at "straight ahead" instead of
   jumping, and the full mechanical range is still reached.               */
#define SERVO_CENTRE_DEAD   100

/* smaller than this and the servo cannot resolve it anyway -- treating it
   as "no change" keeps stick noise from chattering the output (jitter) */
#define SERVO_DEAD_US       20

#define IABS(x)       ((x) < 0 ? -(x) : (x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---------- RC receiver ---------- */
TIM_HandleTypeDef htim1;                /* PA8 = TIM1_CH1, PA9 = TIM1_CH2 */
TIM_HandleTypeDef htim4;                /* PB8 = TIM4_CH3, PB9 = TIM4_CH4 */

volatile uint16_t rc_pulse[5]  = {0, 1500, 1500, 1500, 1500};  /* 1..4 pulse width (us) */
volatile uint8_t  rc_risen[5]  = {0, 0, 0, 0, 0};
volatile uint32_t rc_rise[5]   = {0, 0, 0, 0, 0};
volatile uint32_t rc_last_tick = 0;

uint8_t RXDATA[1];                      /* bluetooth, reserved */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ======================= RC PWM input ======================= */

/* motor table: [0]LF [1]RF [2]RR [3]LR
   fwd pin / rev pin follow the existing wiring (same as old state 1..10) */
typedef struct
{
  uint32_t      ch;
  GPIO_TypeDef *p_fwd;
  uint16_t      pin_fwd;
  GPIO_TypeDef *p_rev;
  uint16_t      pin_rev;
} Motor_t;

static const Motor_t MOT[4] =
{
  { TIM_CHANNEL_1, GPIOA, GPIO_PIN_3,  GPIOA, GPIO_PIN_5  },  /* LF */
  { TIM_CHANNEL_2, GPIOB, GPIO_PIN_1,  GPIOB, GPIO_PIN_0  },  /* RF */
  { TIM_CHANNEL_3, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12 },  /* RR */
  { TIM_CHANNEL_4, GPIOA, GPIO_PIN_10, GPIOA, GPIO_PIN_11 },  /* LR */
};

/* one wheel output, spd = -100..+100 */
static void Motor_Set(uint8_t i, int32_t spd)
{
  if (spd >  100) spd =  100;
  if (spd < -100) spd = -100;

  if (spd > 0)
  {
    HAL_GPIO_WritePin(MOT[i].p_fwd, MOT[i].pin_fwd, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOT[i].p_rev, MOT[i].pin_rev, GPIO_PIN_RESET);
  }
  else if (spd < 0)
  {
    HAL_GPIO_WritePin(MOT[i].p_fwd, MOT[i].pin_fwd, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT[i].p_rev, MOT[i].pin_rev, GPIO_PIN_SET);
    spd = -spd;
  }
  else
  {
    HAL_GPIO_WritePin(MOT[i].p_fwd, MOT[i].pin_fwd, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT[i].p_rev, MOT[i].pin_rev, GPIO_PIN_RESET);
  }

  __HAL_TIM_SetCompare(&htim2, MOT[i].ch, (uint32_t)spd);
}

/* one channel -> -100..+100 */
static int32_t RC_Axis(uint8_t ch)
{
  int32_t p = (int32_t)rc_pulse[ch];
  int32_t v;

  if (p < RC_MIN_US || p > RC_MAX_US) return 0;   /* 900us fail-safe / glitch -> stop */

  v = p - 1500;
  if (v >  500) v =  500;
  if (v < -500) v = -500;

  if (v > -(int32_t)RC_DEADBAND && v < (int32_t)RC_DEADBAND) return 0;

  return v * 100 / 500;
}

/* stick command -> wheel command curve, used on all three driving channels.
   Below 75% of the stick the command is a flat 50%, so the car never creeps
   along in the slow band where the four wheels disagree and it wanders;
   75..100% of the stick then ramps 50..100%.  A stick inside the centre
   dead band is still 0, so the car stops when released. */
static int32_t Stick_Curve(int32_t v)
{
  int32_t d = IABS(v);
  int32_t q;

  if (d == 0) q = 0;                       /* stick centred -> stop       */
  else        q = STICK_MIN_MOVE + d * (100 - STICK_MIN_MOVE) / 100;

  return (v < 0) ? -q : q;
}

/* spin curve, requested separately:
     0 .. 75% of the stick  -> 20% .. 50%  (never slower than 20%, or the
                               wheels would not keep turning)
     75 .. 100% of the stick -> 50% .. 100%
   A stick inside the centre dead band is still 0, so the car stops. */
static int32_t Spin_Curve(int32_t v)
{
  int32_t d = IABS(v);
  int32_t q;

  if (d == 0)       q = 0;
  else if (d <= 75) q = 20 + d * 30 / 75;
  else              q = 50 + (d - 75) * 50 / 25;

  return (v < 0) ? -q : q;
}

/* init TIM1 / TIM4 as 1us input capture */
static void RC_Init(void)
{
  TIM_IC_InitTypeDef ic = {0};

  ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic.ICPrescaler = TIM_ICPSC_DIV1;
  ic.ICFilter    = 3;                     /* fCK_INT, N=8 */

  /* TIM1: PA8 / PA9 */
  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 71;      /* 72MHz/72 = 1MHz -> 1 count = 1us */
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 0xFFFF;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_IC_Init(&htim1);
  HAL_TIM_IC_ConfigChannel(&htim1, &ic, TIM_CHANNEL_1);
  HAL_TIM_IC_ConfigChannel(&htim1, &ic, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

  /* TIM4: PB8 / PB9 */
  htim4.Instance               = TIM4;
  htim4.Init.Prescaler         = 71;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.Period            = 0xFFFF;
  htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_IC_Init(&htim4);
  HAL_TIM_IC_ConfigChannel(&htim4, &ic, TIM_CHANNEL_3);
  HAL_TIM_IC_ConfigChannel(&htim4, &ic, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  rc_last_tick = HAL_GetTick();
}

/* read sticks -> mecanum mixing -> outputs */
static void RC_Control(void)
{
  int32_t vx, vy, wz;
  int32_t lf, rf, rr, lr, mx, yawfix;
  int32_t p, d, us;
  /* -1 = nothing written yet, so the first pass always writes.  Starting
     this at the centre value instead would make the dead band swallow the
     very first update and the servo would get no pulse at all. */
  static int32_t servo_us = -1;   /* last value actually written */
  static uint32_t kick_until = 0; /* start-up boost runs until this tick */
  static uint8_t  kick_idle  = 1; /* the sticks were all centred last pass */

  /* ---- servo ----
     stick down  (1000us) -> 2223us = 80 deg right
     stick centre(1500us) -> 1400us = straight ahead
     stick up    (2000us) ->  572us = 80.5 deg left                */
#if SERVO_INSTALL_MODE
  /* shaft parked on the servo's mechanical centre; the stick is ignored */
  p = (int32_t)rc_pulse[RC_CH_SERVO];
  (void)p;
  us = 1500;
#else
  p = (int32_t)rc_pulse[RC_CH_SERVO];
  if (p < 800 || p > 2200) p = 1500;      /* no signal -> straight ahead */
  if (p < 1000) p = 1000;
  if (p > 2000) p = 2000;

  /* hold the arm perfectly still while the stick sits near the middle --
     this is the only place it hunts.  d is the stick offset from centre
     with the dead zone cut out of it, so d == 0 in the middle and the arm
     leaves the middle from "straight ahead" rather than snapping to it. */
  d = p - 1500;
  if (IABS(d) < SERVO_CENTRE_DEAD) d = 0;
  else if (d > 0)                  d -= SERVO_CENTRE_DEAD;
  else                             d += SERVO_CENTRE_DEAD;

  /* each half of the stick drives its side all the way to the servo's
     limit, so the arm gets every degree the hardware can give */
  if (d < 0)
    us = SERVO_US_ZERO - d * (SERVO_US_MAX - SERVO_US_ZERO) / (500 - SERVO_CENTRE_DEAD);
  else
    us = SERVO_US_ZERO - d * (SERVO_US_ZERO - SERVO_US_MIN) / (500 - SERVO_CENTRE_DEAD);

  us += SERVO_TRIM_US;
  if (us < SERVO_US_MIN) us = SERVO_US_MIN;
  if (us > SERVO_US_MAX) us = SERVO_US_MAX;
#endif

  /* jitter control:
       - a change smaller than SERVO_DEAD_US is stick noise the servo cannot
         resolve anyway, so keep the old value and do not touch the timer;
       - only write CCR when it really changes.  Combined with the CCR
         preload set up in main(), every pulse stays a clean, full-width
         one and the servo stops twitching. */
  if (IABS(us - servo_us) >= SERVO_DEAD_US)
  {
    servo_us = us;
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint32_t)us);
  }

  /* ---- fail-safe ---- */
  if ((HAL_GetTick() - rc_last_tick) > RC_TIMEOUT_MS)
  {
    Motor_Set(0, 0);
    Motor_Set(1, 0);
    Motor_Set(2, 0);
    Motor_Set(3, 0);
    return;
  }

  vx = RC_Axis(RC_CH_FWD);                /* forward / backward */
  vy = RC_Axis(RC_CH_LAT);                /* strafe            */
  wz = RC_Axis(RC_CH_ROT);                /* spin in place     */

  /* same curve on all three driving channels (see Stick_Curve): below 75%
     of the stick the command is a flat 50%, so the car never creeps along in
     the slow band where the four wheels disagree and it wanders. */
  vx = Stick_Curve(vx);
  vy = Stick_Curve(vy);
  wz = Spin_Curve(wz);

  /* start-up kick (see MOT_KICK_MS): a fresh command after standstill gets
     a short burst, so the wheels that need more torque can break away */
  if (vx == 0 && vy == 0 && wz == 0)
  {
    kick_until = 0;
    kick_idle  = 1;
  }
  else if (kick_idle)
  {
    kick_idle  = 0;
    kick_until = HAL_GetTick() + MOT_KICK_MS;
  }

  /* straight-line correction (see MEC_YAW_TRIM / MEC_STRAFE_TRIM):
     - bending: give the left wheels a bit more and the right wheels a bit
       less, proportional to how hard the forward stick is pushed.  It flips
       automatically in reverse, which is what we want.
     - strafing: pull back by a fixed share of the strafe command. */
  /* measured with a very slow stick push, the wheels start in this order:
       front-left, rear-right, rear-left, front-right last
     so the LEFT side breaks away first and the car is pushed left.  Give
     the right wheels a kick that fades out as the stick goes in. */
  if (vx > 0)
    yawfix = -(MEC_KICK * (100 - vx) / 100);
  else if (vx < 0)
    yawfix =  (MEC_KICK * (100 + vx) / 100);
  else
    yawfix =  0;

  vx -= IABS(vy) * MEC_STRAFE_TRIM / 100;

  /* mecanum inverse kinematics (+ = wheel spins forward) */
  lf = vx + vy + wz + yawfix;
  rf = vx - vy - wz - yawfix;
  rr = vx - vy + wz - yawfix;
  lr = vx + vy - wz + yawfix;

  /* scale down so the largest never exceeds 100 */
  mx = IABS(lf);
  if (IABS(rf) > mx) mx = IABS(rf);
  if (IABS(rr) > mx) mx = IABS(rr);
  if (IABS(lr) > mx) mx = IABS(lr);
  if (mx > 100)
  {
    lf = lf * 100 / mx;
    rf = rf * 100 / mx;
    rr = rr * 100 / mx;
    lr = lr * 100 / mx;
  }

  /* start-up boost, applied last so it is not cancelled by the scaling */
  if (HAL_GetTick() < kick_until)
  {
    lf = lf * MOT_KICK_NUM / MOT_KICK_DEN;
    rf = rf * MOT_KICK_NUM / MOT_KICK_DEN;
    rr = rr * MOT_KICK_NUM / MOT_KICK_DEN;
    lr = lr * MOT_KICK_NUM / MOT_KICK_DEN;
  }

  Motor_Set(0, lf);
  Motor_Set(1, rf);
  Motor_Set(2, rr);
  Motor_Set(3, lr);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* servo pulse: enable CCR preload, so a new width written at any moment
     is latched at the start of the next 20ms frame instead of chopping the
     pulse that is currently high.  A chopped pulse is exactly what makes
     the servo twitch while the stick is moving. */
  htim3.Instance->CCMR1 |= TIM_CCMR1_OC1PE;
  htim3.Instance->EGR   |= TIM_EGR_UG;      /* load the value set above */

  HAL_UART_Receive_IT(&huart1, RXDATA, 1);

  RC_Init();          /* start TIM1/TIM4 input capture */
  RC_Control();       /* motors stopped, servo centered */

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    RC_Control();
    HAL_Delay(5);

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ============ RC input capture callback ============ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t ch;          /* 1..4  : index for our arrays          */
  uint32_t chan;        /* TIM_CHANNEL_x : HAL register selector */
  uint32_t val;
  uint32_t w;

  /* htim->Channel uses HAL_TIM_ACTIVE_CHANNEL_x (1,2,4,8),
     while HAL_TIM_ReadCapturedValue / SET_CAPTUREPOLARITY use
     TIM_CHANNEL_x (0,4,8,12). They are NOT the same numbering. */
  switch (htim->Channel)
  {
    case HAL_TIM_ACTIVE_CHANNEL_1: ch = 1; break;
    case HAL_TIM_ACTIVE_CHANNEL_2: ch = 2; break;
    case HAL_TIM_ACTIVE_CHANNEL_3: ch = 3; break;
    case HAL_TIM_ACTIVE_CHANNEL_4: ch = 4; break;
    default: return;
  }
  chan = (ch - 1U) * 4U;                               /* 1->0, 2->4, 3->8, 4->12 */

  val = HAL_TIM_ReadCapturedValue(htim, chan);

  if (rc_risen[ch] == 0)
  {
    rc_rise[ch]  = val;                                /* rising edge seen  */
    rc_risen[ch] = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, chan, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else
  {
    rc_risen[ch] = 0;                                  /* falling edge -> width */
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, chan, TIM_INPUTCHANNELPOLARITY_RISING);

    w = (val - rc_rise[ch]) & 0xFFFFU;                 /* 16-bit wrap-safe  */
    if (w > 700U && w < 2500U)                         /* glitch filter     */
    {
      rc_pulse[ch] = (uint16_t)w;
      rc_last_tick = HAL_GetTick();
    }
  }
}

/* ============ TIM1 / TIM4 low level init ============ */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef g = {0};

  if (htim->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin  = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  }
  else if (htim->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin  = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

/* ============ bluetooth (reserved, keep RX armed) ============ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    RXDATA[0] = 0;
    HAL_UART_Receive_IT(&huart1, RXDATA, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
