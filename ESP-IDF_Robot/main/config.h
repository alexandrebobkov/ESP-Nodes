#ifndef CONFIG_H
#define CONFIG_H

// MOTORS PWM CONFIG
#define MTR_FREQUENCY               (7000)                  // 1000
#define MTR_MODE                    LEDC_LOW_SPEED_MODE
#define MTR_DUTY_RES                LEDC_TIMER_13_BIT       // 13-bit resolution supports maximum duty value 8192 (8)
// LEFT SIDE MOTORS, FORWARD
#define MTR_FRONT_LEFT_IO           (6)
#define MTR_FRONT_LEFT_TMR          LEDC_TIMER_0
#define MTR_FRONT_LEFT              LEDC_CHANNEL_1
#define MTR_FRONT_LEFT_DUTY         (3361)
// RIGHT SIDE MOTORS, FORWARD
#define MTR_FRONT_RIGHT_IO          (5)
#define MTR_FRONT_RIGHT_TMR         LEDC_TIMER_1
#define MTR_FRONT_RIGHT             LEDC_CHANNEL_0
#define MTR_FRONT_RIGHT_DUTY        (3361)
// LEFT SIDE MOTORS, REVERSE
#define MTR_FRONT_LEFT_REV_IO       (4)
#define MTR_FRONT_LEFT_REV_TMR      LEDC_TIMER_2
#define MTR_FRONT_LEFT_REV          LEDC_CHANNEL_2
#define MTR_FRONT_LEFT_REV_DUTY     (3361)
// RIGHT SIDE MOTORS, REVERSE
#define MTR_FRONT_RIGHT_REV_IO      (7)
#define MTR_FRONT_RIGHT_REV_TMR     LEDC_TIMER_3
#define MTR_FRONT_RIGHT_REV         LEDC_CHANNEL_3
#define MTR_FRONT_RIGHT_REV_DUTY    (3361)


//#define LEDC_DUTY                   (3361) //7820) // 8068, 7944, 7820, 7696, 7572, *7680*, 7424, 7168, 6144, 512, 768
//#define LEDC_FREQUENCY              (2500) //8192) //4000) // For LED the freuqncy of 500Hz seems to be sufficient. // Frequency in Hertz. For DC motor, set frequency at 5 kHz; try 1kHz @ 14 bits resolution


#endif
