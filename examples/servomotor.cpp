#include <ServoMotor.h>

#include <stdint.h>

int main() {
    // Initialize servos (use default constructor)
    ServoMotor servo1;

    // Attach servo to specific port and pin
    servo1.attach(&PORTA, PINA1);

    // Attach servo to specific port and pin with pass calibration limits
    servo1.attach(&PORTA, PINA1, 600, 2000);

    // Or set calibration later
    servo1.setMIN(540);
    servo1.setMAX(2500);

    // Get current calibration values
    uint16_t min = servo1.getMIN();
    uint16_t max = servo1.getMAX();

    // Get servo angle as degree
    uint16_t degree = servo1.getAngle();

    // Get servo angle as microseconds
    uint16_t ms = servo1.getMicroseconds();

    // Set servo angle as degree
    servo1.setAngle(45);

    // Set servo angle as microseconds
    servo1.setMicroseconds(1000);

    // Set angle with raise duration
    servo1.setAngle(45, 5);
    servo1.setMicroseconds(1000, 5);

    // Detach servo to release timer
    servo1.detach();
}

