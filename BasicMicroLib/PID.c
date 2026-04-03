#include "PID.h"

float PID_calculate(PID* pid,float new_Value,float old_Value){
    pid->saved_i+=pid->i*(new_Value);
    if (pid->saved_i>pid->i_Max)pid->saved_i=pid->i_Max;
    if(pid->saved_i<(-pid->i_Max))pid->saved_i = -pid->i_Max;
    return (pid->p*(new_Value))+(pid->saved_i)+(pid->d*(new_Value-old_Value));
}