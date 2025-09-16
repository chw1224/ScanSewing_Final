#include "encoderMonitor.h"
#include "Arduino.h"

encoderMonitor::encoderMonitor(char axisName, int step_per_rot, unsigned long origin) {
    axis = axisName;
    step_per_rotation = step_per_rot;
    origin = origin;
    position = 0;
}

encoderMonitor::~encoderMonitor() {
}

void encoderMonitor::pinInterrA(void) {
    encoderStatusArr[pinA] = digitalRead(pinArr[pinA]);
    encoderStatusArr[pinB] = digitalRead(pinArr[pinB]);
    
    if(encoderStatusArr[pinA] == HIGH) {
        if(encoderStatusArr[pinB] == LOW) {
            changePosition(CW);
        }
        else if(encoderStatusArr[pinB] == HIGH) {
            changePosition(CCW);
        }
    }
    else if(encoderStatusArr[pinA] == LOW) {
        if(encoderStatusArr[pinB] == HIGH) {
            changePosition(CW);
        }
        else if(encoderStatusArr[pinB] == HIGH) {
            changePosition(CCW);
        }
    }

    if(axis == 'Z') {
        if(position == step_per_rotation) {
            resetPosition();
        }
        // if(position > step_per_rotation) {
        //     position = step_per_rotation;
        // }
    }
}

void encoderMonitor::pinInterrB(void) {
    encoderStatusArr[pinA] = digitalRead(pinArr[pinA]);
    encoderStatusArr[pinB] = digitalRead(pinArr[pinB]);

    if(encoderStatusArr[pinB] == HIGH) {
        if(encoderStatusArr[pinA] == HIGH) {
            changePosition(CW);
        }
        else if(encoderStatusArr[pinA] == LOW) {
            changePosition(CCW);
        }
    }
    else if(encoderStatusArr[pinB] == LOW) {
        if(encoderStatusArr[pinA] == LOW) {
            changePosition(CW);
        }
        else if(encoderStatusArr[pinA] == HIGH) {
            changePosition(CCW);
        }
    }

    if(axis == 'Z') {
        if(position == step_per_rotation) {
            resetPosition();
        }
        // if(position > step_per_rotation) {
        //     position = step_per_rotation;
        // }
    }
}

void encoderMonitor::setEncoderPin(int encPinA, int encPinB) {
    pinArr[pinA] = encPinA;
    pinArr[pinB] = encPinB;
}

void encoderMonitor::init(void) {
    position = 0;
    
    pinMode(pinArr[pinA], INPUT_PULLUP);
    pinMode(pinArr[pinB], INPUT_PULLUP);

    encoderStatusArr[pinA] = digitalRead(pinArr[pinA]);
    encoderStatusArr[pinB] = digitalRead(pinArr[pinB]);
}

void encoderMonitor::setPosition(unsigned long initPos) {
    position = initPos;
}

void encoderMonitor::resetPosition(void) {
    position = origin;
}

void encoderMonitor::changePosition(int rotDir) {
    switch(rotDir) {
        case CW:
            position++;
            break;
        case CCW:
            position--;
            break;
    }
}

// char* encoderMonitor::message(void) {
//     memset((void *)messageBuffer, '\0', MESSAGE_MAX_LENGH);
//     sprintf(messageBuffer, "%c%lu", axis, position);

//     return messageBuffer;
// }

unsigned long* encoderMonitor::message(void) {
    return &position;
}