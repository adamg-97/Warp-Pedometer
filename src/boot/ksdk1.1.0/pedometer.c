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
#define THRESH                  2000            // Step threshold for LPF data
#define DERIV_THRESH            250             // Step threshold for deriv data
#define RUNNING_THRESH          8               // Threshold for running mode - 8 steps in 3s = 2.66Hz
#define REST_TIME               2500            // Enter rest mode after 5s

int16_t     diff_coeff[BUFF_LENGTH]     =   {0,0,0,1,0,-1,0,0,0};               // FIR derivative
int16_t     lpf_coeff[BUFF_LENGTH]      =   {1,6,22,44,54,44,22,6,1};           // FIR LPF filter designed using MATLAB

uint32_t    data_buff[BUFF_LENGTH]      =   {0};                                // Data buffer
uint32_t    lpf_buff[BUFF_LENGTH]       =   {0};                                // Buffer to store lpf signal
int32_t     deriv_buff[BUFF_LENGTH]     =   {0};                                // Buffer to store derivative of signal
bool        step_buff[STEP_BUFF_LENGTH] =   {0};                                // Buffer to hold recent previous steps

int8_t      n                           =   BUFF_LENGTH - 1;                    // Index of last number in buffer
uint8_t steps_in_buffer                 =   0;                                  // Keep track of how many steps in buffer for mode selection



// Combine the stream from x,y,z by squaring, adding and square-rooting
int16_t  combine_stream(int16_t x_data, int16_t y_data, int16_t z_data){
    
    int16_t comb_data = (int16_t)sqrt(x_data*x_data + y_data*y_data + z_data*z_data);
    
    SEGGER_RTT_printf(0, "%d, ", x_data);
    SEGGER_RTT_printf(0, "%d, ", y_data);
    SEGGER_RTT_printf(0, "%d, ", z_data);
    SEGGER_RTT_printf(0, "%d\n", comb_data);
    
    
    return comb_data;
}


// FIR Low Pass Filter
void lpf(void){
    
    uint32_t moving_lpf = 0;

    for(uint8_t i = 0; i < BUFF_LENGTH; i++){
            moving_lpf += lpf_coeff[i]*data_buff[n - i];
    }
    // Store in LPF buffer
    lpf_buff[n] = moving_lpf / 8;       // Divide to avoid overflow - resolution ok for step counting
}


// FIR derivative
void  diff(void){
    
    int32_t     moving_deriv = 0;

    for(uint8_t i = 0; i < BUFF_LENGTH; i++){
            moving_deriv += diff_coeff[i]*lpf_buff[n - i];
    }
    // Store in derivative buffer
    deriv_buff[n] = moving_deriv;
}

// Main function to count steps
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
        step_count ++;
        step_buff[STEP_BUFF_LENGTH - 1] = 1;
    }
    else{
        step_buff[STEP_BUFF_LENGTH - 1] = 0;
    }
    
    // Keep track of steps in buffer based on entering and leaving steps
    steps_in_buffer = steps_in_buffer + step_buff[STEP_BUFF_LENGTH - 1] - step_buff[0];
    
    return step_count;
}


// Count calories based on speed, weight and height
uint32_t countCals(uint32_t cal_count, uint8_t height, uint8_t weight)
{
    
    // Uses equations outlined in 'Full-Featured Pedometer Design Realized with 3-Axis Digital Accelerometer' - Neil Zhao
    
    uint8_t cals;
    uint8_t speed;
    uint8_t stride;
    
    
    // Calculate stride length from steps per 3s
    if(steps_in_buffer < 3)
    {
        stride = height / 5;
    }
    
    else if(steps_in_buffer < 4)
    {
        stride = height / 4;
    }
    
    else if(steps_in_buffer < 6)
    {
        stride = height / 3;
    }
    
    else if(steps_in_buffer < 7)
    {
        stride = height / 2;
    }
    
    else if(steps_in_buffer < 9)
    {
        stride = height / 1.2;
    }
    
    else if(steps_in_buffer < 12)
    {
        stride = height;
    }
    
    else
    {
        stride = height * 1.2;
    }
    
    // Calculate speed in cm/s
    speed = steps_in_buffer * stride / 3;       // Speed = number of steps * stride length / time (3s)
    
    /*
    Calculate calories per second (cals used i.e. Kcals x1000)
    kCals/kg/h = 1.25 x running speed (km/h)
    Cals/s = 1.25 * weight (kg) * running speed (m/s) = 1.25 * weight * running speed (cm/s) / 100
    */
    
    cals = 1.25 * weight * speed / 100;
    
    cal_count += cals;
    
    return(cal_count);
}

// Select mode based on steps
uint8_t modeSelector(uint8_t mode, uint32_t last_step_time)
{
    // Rest mode if no steps in REST_TIME
    if(OSA_TimeGetMsec() - last_step_time > REST_TIME)
    {
        return 0;
    }
    // Running mode if number of steps in buffer > RUNNING_THRESH
    else if(steps_in_buffer > RUNNING_THRESH)
    {
        return 2;
    }
    // Otherwise walking mode
    else
    {
        return 1;
    }
}

