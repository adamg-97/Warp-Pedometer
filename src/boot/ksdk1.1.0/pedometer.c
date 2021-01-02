/*
    Authored 2020-2021. Adam Goldney
 */

#include <stdlib.h>
#include <math.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"

#define BUFF_LENGTH             9
#define STEP_BUFF_LENGTH        150             // Record steps for last 3s for mode selection
#define THRESH                  2000
#define DERIV_THRESH            250
#define RUNNING_THRESH          8               // 8 steps in 3s = 2.66Hz
#define REST_TIME               5000            // Rest after 5s

int16_t     diff_coeff[BUFF_LENGTH]     =   {0,0,0,1,0,-1,0,0,0};               // FIR derivative
int16_t     lpf_coeff[BUFF_LENGTH]      =   {1,6,22,44,54,44,22,6,1};           // Designed using MATLAB

uint32_t    data_buff[BUFF_LENGTH]      =   {0};                                // Data buffer
uint32_t    lpf_buff[BUFF_LENGTH]       =   {0};                                // Buffer to store lpf signal
int32_t     deriv_buff[BUFF_LENGTH]     =   {0};                                // Buffer to store derivative of signal
bool        step_buff[STEP_BUFF_LENGTH] =   {0};                                // Buffer to hold recent previous steps

int8_t      n                           =   BUFF_LENGTH - 1;                    // Index of last number in buffer
uint8_t steps_in_buffer                 =   0;                                  // Keep track of how many steps in buffer for mode selection



// Combine the stream from x,y,z by squaring, adding and square-rooting
int16_t  combine_stream(int16_t x_data, int16_t y_data, int16_t z_data){
    
    int16_t comb_data = (int16_t)sqrt(x_data*x_data + y_data*y_data + z_data*z_data);
    
    return comb_data;
}


// FIR Low Pass Filter
void lpf(void){
    
    uint32_t moving_lpf = 0;

    for(uint8_t i = 0; i < BUFF_LENGTH; i++){
            moving_lpf += lpf_coeff[i]*data_buff[n - i];
    }
    lpf_buff[n] = moving_lpf / 8;
}


// FIR derivative
void  diff(void){
    
    int32_t     moving_deriv = 0;

    for(uint8_t i = 0; i < BUFF_LENGTH; i++){
            moving_deriv += diff_coeff[i]*(int16_t)lpf_buff[n - i];
    }
    deriv_buff[n] = moving_deriv;
}


uint32_t countSteps(uint32_t step_count){

    // Shift the elements in the buffers left to the left
    for(int i=0; i < STEP_BUFF_LENGTH - 1; i++){
        
        if(i < n)
        {
            data_buff[i]    = data_buff[i+1];
            lpf_buff[i]     = lpf_buff[i+1];
            deriv_buff[i]   = deriv_buff[i+1];
        }
        
        step_buff[i]    = step_buff[i+1];

    }


    // Set last element in array to new data point
    data_buff[n] = combine_stream(readAxis_x(),readAxis_y(),readAxis_z());
    
    // Low pass filter the data
    lpf();
    
    // Differentiate
    diff();
    

    // Check there haven't been any steps in the last buffer period (as repeat counts in the period are too fast to be additional steps)
    bool recent_step = 0;
    
    for(int i=1; i <= BUFF_LENGTH; i++){
        if(step_buff[STEP_BUFF_LENGTH - i] != 0){
            recent_step = 1;
            break;
        }
    }
    
    // Count the steps using derivative and spread of data points
    if((deriv_buff[n] * deriv_buff[n-1]) < 0 && lpf_buff[n-4] > THRESH && (deriv_buff[n-4] - deriv_buff[n]) > DERIV_THRESH && recent_step == 0){
        //SEGGER_RTT_printf(0, "%d\n", 1);
        step_count ++;
        step_buff[STEP_BUFF_LENGTH - 1] = 1;
    }
    else{
        //SEGGER_RTT_printf(0, "%d\n", 0);
        step_buff[STEP_BUFF_LENGTH - 1] = 0;
    }
    return step_count;
}



uint32_t countCals(uint32_t cal_count, uint8_t height, uint8_t weight, uint8_t mode)
{
    uint8_t step_cals;
    uint8_t multiplier;
    
    if((mode == 1) || (mode == 0))
    {
        multiplier = 1;
    }
    else if (mode == 2)
    {
        multiplier = 1.5;
    }
    
    // Calculate calories per step - note this is in cals not Kcals so /1000 for Kcals
    step_cals = weight * height * multiplier * 0.025;
    
    cal_count += step_cals;
    
    return(cal_count);
}


uint8_t modeSelector(uint8_t mode, uint32_t last_step_time)
{
    // Keep track of steps in buffer based on entering and leaving steps
    steps_in_buffer = steps_in_buffer + step_buff[STEP_BUFF_LENGTH - 1] - step_buff[0];

    if(OSA_TimeGetMsec() - last_step_time > REST_TIME)
    {
        return 0;
    }
    else if(steps_in_buffer > RUNNING_THRESH)
    {
        return 2;
    }
    else
    {
        return 1;
    }
}

