#include "ServoMotor.h"

// Dependency on Timer lib
#include <Timer16Bit.h>

#if defined(TCNT1) && SERVOMOTOR_USE_TIMER1
#include <Timer1.h>
#endif

#if defined(TCNT3) && SERVOMOTOR_USE_TIMER3
#include <Timer3.h>
#endif

#if defined(TCNT4) && SERVOMOTOR_USE_TIMER4
#include <Timer4.h>
#endif

#if defined(TCNT5) && SERVOMOTOR_USE_TIMER5
#include <Timer5.h>
#endif

#include <util/atomic.h>
#include <stdint.h>
#include <stdlib.h>

#define TICKS_PER_US() (F_CPU / 1000000L)
#define US_TO_TICKS(_val_) ((uint16_t) (_val_ * TICKS_PER_US() / 8))
#define TICKS_TO_US(_val_) ((uint16_t) (_val_ * 8 / TICKS_PER_US()))
#define REFRESH_US 20000U
#define REFRESH_FPS 50U
#define CONSTRAIN(_val_, _min_, _max_) (_min_ > _val_ ? _min_ : (_max_ < _val_ ? _max_ : _val_))

#define SERVOMOTOR_INDEX_TO_TIMER(_index_) ((ServomotorTimer) (_index_ / SERVOMOTOR_PER_TIMER))
#define SERVOMOTOR_INDEX_BY_TIMER(_timerN_, _channel_) ((_timerN_ * SERVOMOTOR_PER_TIMER) + _channel_)

typedef struct {
    uint8_t number: 3;
    uint8_t attached: 1;
} pin_t;

typedef struct {
    volatile uint8_t *port;
    pin_t pin;
    uint16_t ticks;
    uint16_t target;
    uint16_t step;
} servo_t;

static volatile uint8_t count = 0;
static volatile servo_t servos[SERVOMOTOR_TOTAL];
static volatile int8_t channels[SERVOMOTOR_TIMER_COUNT];

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline void onTimerCompareA(ServomotorTimer timerN, Timer16BitClass *timer) {
    uint8_t index;

    if (channels[timerN] < 0) {
        *timer->TCNTn = 0;
    } else {
        index = SERVOMOTOR_INDEX_BY_TIMER(timerN, channels[timerN]);

        if (index < count && servos[index].pin.attached) {
            *(servos[index].port) &= ~_BV(servos[index].pin.number);
        }
    }

    channels[timerN]++;

    index = SERVOMOTOR_INDEX_BY_TIMER(timerN, channels[timerN]);

    if (index < count && channels[timerN] < SERVOMOTOR_PER_TIMER) {
        if (servos[index].step > 0) {
            if (servos[index].target > servos[index].ticks) {
                servos[index].ticks += servos[index].step;

                if (servos[index].target <= servos[index].ticks) {
                    servos[index].ticks = servos[index].target;
                    servos[index].step  = 0;
                }
            } else {
                servos[index].ticks -= servos[index].step;

                if (servos[index].target >= servos[index].ticks) {
                    servos[index].ticks = servos[index].target;
                    servos[index].step  = 0;
                }
            }
        }

        *timer->OCRnA = *timer->TCNTn + servos[index].ticks;

        if (servos[index].pin.attached) {
            *(servos[index].port) |= _BV(servos[index].pin.number);
        }
    } else {
        if (*timer->TCNTn + 4 < US_TO_TICKS(REFRESH_US)) {
            *timer->OCRnA = (uint16_t) US_TO_TICKS(REFRESH_US);
        } else {
            *timer->OCRnA = *timer->TCNTn + 4;
        }

        channels[timerN] = -1;
    }
}

static void interruptEnable(ServomotorTimer timerN) {
#if defined(TCNT1) && SERVOMOTOR_USE_TIMER1
    if (SERVOMOTOR_TIMER1 == timerN) {
        Timer1.setCountMode(TIMER_16BIT_COUNT_NORMAL);
        Timer1.setClockSource(TIMER_CLOCK_DIVIDE_BY_8);
        Timer1.setInterruptHandler(TIMER_INTERRUPT_COMPARE_MATCH_A, [](){ onTimerCompareA(SERVOMOTOR_TIMER1, &Timer1); });
        Timer1.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, true);
    }
#endif

#if defined(TCNT3) && SERVOMOTOR_USE_TIMER3
    if (SERVOMOTOR_TIMER3 == timerN) {
        Timer3.setCountMode(TIMER_16BIT_COUNT_NORMAL);
        Timer3.setClockSource(TIMER_CLOCK_DIVIDE_BY_8);
        Timer3.setInterruptHandler(TIMER_INTERRUPT_COMPARE_MATCH_A, [](){ onTimerCompareA(SERVOMOTOR_TIMER3, &Timer3); });
        Timer3.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, true);
    }
#endif

#if defined(TCNT4) && SERVOMOTOR_USE_TIMER4
    if (SERVOMOTOR_TIMER4 == timerN) {
        Timer4.setCountMode(TIMER_16BIT_COUNT_NORMAL);
        Timer4.setClockSource(TIMER_CLOCK_DIVIDE_BY_8);
        Timer4.setInterruptHandler(TIMER_INTERRUPT_COMPARE_MATCH_A, [](){ onTimerCompareA(SERVOMOTOR_TIMER4, &Timer4); });
        Timer4.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, true);
    }
#endif

#if defined(TCNT5) && SERVOMOTOR_USE_TIMER5
    if (SERVOMOTOR_TIMER5 == timerN) {
        Timer5.setCountMode(TIMER_16BIT_COUNT_NORMAL);
        Timer5.setClockSource(TIMER_CLOCK_DIVIDE_BY_8);
        Timer5.setInterruptHandler(TIMER_INTERRUPT_COMPARE_MATCH_A, [](){ onTimerCompareA(SERVOMOTOR_TIMER5, &Timer5); });
        Timer5.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, true);
    }
#endif
    (void) timerN;
}

static void interruptDisable(ServomotorTimer timerN) {
#if defined(TCNT1) && SERVOMOTOR_USE_TIMER1
    if (SERVOMOTOR_TIMER1 == timerN) {
        Timer1.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, false);
    }
#endif

#if defined(TCNT3) && SERVOMOTOR_USE_TIMER3
    if (SERVOMOTOR_TIMER3 == timerN) {
        Timer3.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, false);
    }
#endif

#if defined(TCNT4) && SERVOMOTOR_USE_TIMER4
    if (SERVOMOTOR_TIMER4 == timerN) {
        Timer4.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, false);
    }
#endif

#if defined(TCNT5) && SERVOMOTOR_USE_TIMER5
    if (SERVOMOTOR_TIMER5 == timerN) {
        Timer5.setInterruptEnabled(TIMER_INTERRUPT_COMPARE_MATCH_A, false);
    }
#endif
    (void) timerN;
}

static bool isTimerActive(ServomotorTimer timerN) {
    if (timerN >= SERVOMOTOR_TIMER_COUNT) {
        return false;
    }

    for (uint8_t channel = 0; channel < SERVOMOTOR_PER_TIMER; channel++) {
        uint8_t index = SERVOMOTOR_INDEX_BY_TIMER(timerN, channel);

        if (servos[index].pin.attached) {
            return true;
        }
    }

    return false;
}

ServoMotor::ServoMotor() {
    if (count < SERVOMOTOR_TOTAL) {
        this->index = count++;
        servos[this->index].ticks = US_TO_TICKS(SERVOMOTOR_PULSE_MID);
    } else {
        this->index = SERVOMOTOR_INVALID ;
    }
}

uint8_t ServoMotor::attach(volatile uint8_t *port, uint8_t pin) {
    return this->attach(port, pin, SERVOMOTOR_PULSE_MIN, SERVOMOTOR_PULSE_MAX);
}

uint8_t ServoMotor::attach(volatile uint8_t *port, uint8_t pin, uint16_t min, uint16_t max) {
    if (this->index < SERVOMOTOR_TOTAL) {
        this->min = min;
        this->max = max;

        servos[this->index].port = port;

        servos[this->index].pin.number   = pin;

        ServomotorTimer timer = SERVOMOTOR_INDEX_TO_TIMER(this->index);
        if (!isTimerActive(timer)) {
            interruptEnable(timer);
        }

        servos[this->index].pin.attached = 1; // <-- This must be last
    }

    return this->index;
}

void ServoMotor::detach() {
    servos[this->index].pin.attached = 0; // <-- This must be first

    ServomotorTimer timer = SERVOMOTOR_INDEX_TO_TIMER(this->index);
    if (!isTimerActive(timer)) {
        interruptDisable(timer);
    }
}

uint16_t ServoMotor::getMIN() {
    return this->min;
}

void ServoMotor::setMIN(uint16_t value) {
    this->min = value;
}

uint16_t ServoMotor::getMAX() {
    return this->max;
}

void ServoMotor::setMAX(uint16_t value) {
    this->max = value;
}

uint16_t ServoMotor::getAngle() {
    return (uint16_t) map(this->getMicroseconds(), this->min, this->max, 0, 180);
}

void ServoMotor::setAngle(uint16_t value) {
    value = CONSTRAIN(value, 0, 180);
    this->setMicroseconds((uint16_t) map(value, 0, 180, this->min, this->max));
}

void ServoMotor::setAngle(uint16_t value, uint8_t duration) {
    value = CONSTRAIN(value, 0, 180);
    this->setMicroseconds((uint16_t) map(value, 0, 180, this->min, this->max), duration);
}

uint16_t ServoMotor::getMicroseconds() {
    if (this->index != SERVOMOTOR_INVALID) {
        return TICKS_TO_US(servos[this->index].ticks);
    } else {
        return 0;
    }
}

void ServoMotor::setMicroseconds(uint16_t value) {
    if (this->index != SERVOMOTOR_INVALID) {
        value = CONSTRAIN(value, this->min, this->max);
        value = US_TO_TICKS(value);

        ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
            servos[this->index].ticks = value;
        }
    }
}

void ServoMotor::setMicroseconds(uint16_t value, uint8_t duration) {
    if (this->index != SERVOMOTOR_INVALID) {
        value = CONSTRAIN(value, this->min, this->max);
        value = US_TO_TICKS(value);

        uint16_t step = abs(servos[this->index].ticks - value) / (REFRESH_FPS * duration);

        ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
            servos[this->index].target = value;
            servos[this->index].step   = step;
        }
    }
}
