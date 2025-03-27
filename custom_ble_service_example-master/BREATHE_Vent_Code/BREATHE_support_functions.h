/* Header file containing the function declarations for the motor controls */

/* Variables for BLE */

/* Includes and variables for the ADC */
// #include "nrf_drv_saadc.h"
// #include "nrf_drv_timer.h"
// #include "nrf_ppi.h"
// #include "nrf_drv_ppi.h"
// #define SAMPLES_IN_BUFFER 5
// volatile uint8_t state = 1;


// const nrf_drv_timer_t m_timer;
// nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
// nrf_ppi_channel_t m_ppi_channel;
// uint32_t m_adc_evt_counter;

// extern const nrf_drv_timer_t m_timer;
// extern nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
// extern nrf_ppi_channel_t     m_ppi_channel;
// extern uint32_t              m_adc_evt_counter;


ble_cus_t* get_ble_cus_instance(void);

void enable_pwm(void); 
void disable_pwm(void);
void motor_switch_enable(void);
void motor_switch_disable(void);
void move_vent (uint32_t angle);
// void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
