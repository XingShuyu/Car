#include "PID.h"

float PID_calculate(PID* pid,float new_Value,float old_Value,float prevOldValue){
    pid->saved_i=pid->i*(new_Value)*pid->t/1000;
    if (pid->saved_i>pid->i_Max)pid->saved_i=pid->i_Max;
    if(pid->saved_i<(-pid->i_Max))pid->saved_i = -pid->i_Max;
    float p = (pid->p*(new_Value-old_Value));
    p+=1;
    p-=1;
    float d = (pid->d*(new_Value-2*old_Value+prevOldValue)/pid->t*1000);
    return (p+(pid->saved_i)+d);
}