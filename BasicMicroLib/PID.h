#ifndef PID_H
#define PID_H

typedef struct{
    float p;
    float i;
    float d;
    float i_Max;
    float saved_i;
    float t;
}PID;

float PID_calculate(PID* pid,float new_Value,float old_Value,float prevOldValue);
#endif // PID_H