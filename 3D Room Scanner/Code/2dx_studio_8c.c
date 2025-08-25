/*  
 * Time-of-Flight (ToF) Sensor Interface for COMPENG 2DX4 – Studio W8-0
 *
 * This program demonstrates integration of the VL53L1X ToF distance sensor with the TM4C1294NCPDT
 * microcontroller. It configures the I²C bus, GPIO ports, and supporting peripherals to collect and 
 * transmit range measurements while controlling a stepper motor to perform angular scanning.
 *
 * Key Features:
 *  - Low-level I²C implementation based on the MSP432E4 Reference Manual (Chapter 19).
 *  - Integration of STMicroelectronics VL53L1X sensor via the Ultra Light Driver (pg. 19–21, VL53L1X datasheet).
 *  - Stepper motor control for rotating the sensor to collect 3D room mapping data.
 *  - User control through push buttons and LED feedback.
 *  - UART output for data logging.
 *
 * Modification History:
 *  - Written by Tom Doyle (2020)
 *  - Updated by Hafez Mousavi Garmaroudi (2020–2022) – added I²C improvements and Keil IDE compatibility.
 *  - Modified by Diego Verdin (Mar 14, 2025) – adapted as a blueprint for final project implementation.
 *
 * Demonstrated Skills:
 *  - Embedded C programming
 *  - Peripheral driver development (I²C, GPIO, UART, SysTick)
 *  - Sensor integration and motor control
 *  - Real-time system design for data acquisition
 */

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

// ======================= I²C Control Bit Definitions =======================
#define I2C_MCS_ACK     0x08  // Acknowledge Data
#define I2C_MCS_ADRACK  0x04  // Acknowledge Address
#define I2C_MCS_STOP    0x04  // Generate STOP condition
#define I2C_MCS_START   0x02  // Generate START condition
#define I2C_MCS_ERROR   0x02  // Error detected
#define I2C_MCS_RUN     0x01  // I2C Master Enable
#define I2C_MCS_BUSY    0x01  // I2C Busy Flag
#define I2C_MCR_MFE     0x10  // I2C Master Function Enable

#define MAXRETRIES 5          // Number of retries before aborting an I²C transaction

// ======================= Global Control Flags =======================
int acquisition_enabled = 0;   // Toggle: Enable/disable ToF data acquisition
int motor_running = 0;         // Toggle: Enable/disable stepper motor
int spin_direction = 1;        // Motor spin direction (1 = CW, 0 = CCW)

/**
 * @brief Initialize I²C0 on Port B (PB2 = SCL, PB3 = SDA).
 * Configures clock gating, alternate functions, open-drain, and speed.
 */
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;     // Activate I²C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;   // Activate Port B
  while((SYSCTL_PRGPIO_R & 0x0002) == 0){};  // Wait until ready

  // Configure PB2, PB3 for I²C alternate function
  GPIO_PORTB_AFSEL_R |= 0x0C;
  GPIO_PORTB_ODR_R   |= 0x08;                // SDA (PB3) open-drain
  GPIO_PORTB_DEN_R   |= 0x0C;
  GPIO_PORTB_AMSEL_R &= ~0x0C;
  GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;

  I2C0_MCR_R = I2C_MCR_MFE;                  // Enable I²C master
  I2C0_MTPR_R = 0x3B;                        // Configure for 100 kbps
}

/**
 * @brief Configure Port J (PJ0, PJ1) for push buttons with pull-ups.
 */
void PortJ_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};
  GPIO_PORTJ_DIR_R &= ~0x03;   // Inputs
  GPIO_PORTJ_DEN_R |= 0x03;
  GPIO_PORTJ_PUR_R |= 0x03;    // Enable pull-ups
}

/**
 * @brief Configure Port E (PE0) for heartbeat LED.
 */
void PortE_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0){};
  GPIO_PORTE_DIR_R  = 0x01;
  GPIO_PORTE_DEN_R  = 0x01;
}

/**
 * @brief Configure Port F (PF0, PF4) for status LEDs.
 */
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
  GPIO_PORTF_DIR_R  |= 0x11;
  GPIO_PORTF_DEN_R  |= 0x11;
  GPIO_PORTF_AMSEL_R &= ~0x11;
}

/**
 * @brief Configure Port N (PN1) for motor status LED.
 */
void PortN_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
  GPIO_PORTN_DIR_R  |= 0x02;
  GPIO_PORTN_DEN_R  |= 0x02;
}

/**
 * @brief Configure Port M (PM0–PM3) for stepper motor outputs.
 */
void PortM_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
  GPIO_PORTM_DIR_R   |= 0x0F;
  GPIO_PORTM_AFSEL_R &= ~0x0F;
  GPIO_PORTM_DEN_R   |= 0x0F;
  GPIO_PORTM_AMSEL_R &= ~0x0F;
}

/**
 * @brief Configure Port G0 for sensor shutdown (XSHUT pin).
 * Driving PG0 low forces VL53L1X into standby.
 */
void PortG_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){};
  GPIO_PORTG_DIR_R   &= 0x00;
  GPIO_PORTG_AFSEL_R &= ~0x01;
  GPIO_PORTG_DEN_R   |= 0x01;
  GPIO_PORTG_AMSEL_R &= ~0x01;
}

/**
 * @brief Reset the VL53L1X sensor via XSHUT pin (PG0).
 */
void VL53L1X_XSHUT(void){
  GPIO_PORTG_DIR_R  |= 0x01;        // PG0 as output
  GPIO_PORTG_DATA_R &= ~0x01;       // Drive low
  FlashAllLEDs();
  SysTick_Wait10ms(10);
  GPIO_PORTG_DIR_R  &= ~0x01;       // Return to Hi-Z
}

/**
 * @brief Rotate stepper motor by a specified number of steps.
 * @param direction  Spin direction (1 = CW, 0 = CCW).
 * @param steps      Number of steps to move.
 */
void spin_degrees(int direction, int steps){
  const uint8_t step_sequence[4] = {0x0C, 0x06, 0x03, 0x09};
  static int step_index = 0;
  int i;
  for ( i = 0; i < steps; i++){
    step_index = (direction == 1) ? (step_index + 1) % 4 :
                                    (step_index == 0 ? 3 : step_index - 1);
    GPIO_PORTM_DATA_R = step_sequence[step_index];
    SysTick_Wait10ms(1);
  }
}

/**
 * @brief Handle user button presses.
 *  - Button 1 (PJ0): Toggle acquisition, status LED on PF4.
 *  - Button 2 (PJ1): Toggle motor, status LED on PN1.
 */
void Buttons(void){
  // Toggle acquisition
  if ((GPIO_PORTJ_DATA_R & 0x01) == 0){
    SysTick_Wait10ms(3);
    while ((GPIO_PORTJ_DATA_R & 0x01) == 0);
    acquisition_enabled ^= 1;
    GPIO_PORTF_DATA_R ^= 0x10;
  }

  // Toggle motor
  if ((GPIO_PORTJ_DATA_R & 0x02) == 0){
    SysTick_Wait10ms(3);
    while ((GPIO_PORTJ_DATA_R & 0x02) == 0);
    motor_running ^= 1;
    if (motor_running) GPIO_PORTN_DATA_R |= 0x02;
    else {
      GPIO_PORTN_DATA_R &= ~0x02;
      GPIO_PORTM_DATA_R  = 0x00;
    }
  }
}

// ======================= MAIN FUNCTION =======================
uint16_t dev = 0x29;  // VL53L1X I²C address
int status = 0;

int main(void){
  uint8_t byteData, sensorState = 0, dataReady;
  uint16_t wordData, Distance;
  uint8_t RangeStatus;

  // Initialize system clock, peripherals, and I/O
  PLL_Init();
  SysTick_Init();
  PortJ_Init();
  PortF_Init();
  PortN_Init();
  PortE_Init();
  PortM_Init();
  onboardLEDs_Init();
  I2C_Init();
  UART_Init();

  UART_printf("Program Begins\r\n");

  // Verify sensor presence
  status = VL53L1X_GetSensorId(dev, &wordData);
  sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);
  UART_printf(printf_buffer);

  // Wait until sensor boots
  while(sensorState == 0){
    status = VL53L1X_BootState(dev, &sensorState);
    SysTick_Wait10ms(10);
  }
  FlashAllLEDs();
  UART_printf("ToF Chip Booted! Please Wait...\r\n");

  // Sensor configuration
  VL53L1X_ClearInterrupt(dev);
  VL53L1X_SensorInit(dev);
  VL53L1X_SetTimingBudgetInMs(dev, 20);
  VL53L1X_SetInterMeasurementInMs(dev, 21);
  VL53L1X_StartRanging(dev);

  int counter = 0;

  // Main loop
  while(1){
    Buttons();
    SysTick_Wait(12000000);   // Heartbeat ~1 sec at 12 MHz
    GPIO_PORTE_DATA_R ^= 0x01;

    if (!motor_running){ 
      GPIO_PORTN_DATA_R &= ~0x02;
      continue;
    }

    int step;
    // 32 steps = full rotation (11.25° each)
    for ( step = 0; step < 32; step++){
      Buttons();
      if (!motor_running) break;

      if (acquisition_enabled) {
        int angle = step * 11.25;
        dataReady = 0;

        // Wait for sensor data
        while (dataReady == 0){
          Buttons();
          if (!motor_running || !acquisition_enabled) break;
          VL53L1X_CheckForDataReady(dev, &dataReady);
          VL53L1_WaitMs(dev, 5);
        }
        if (!motor_running || !acquisition_enabled) break;

        // Collect distance measurement
        VL53L1X_GetRangeStatus(dev, &RangeStatus);
        VL53L1X_GetDistance(dev, &Distance);
        VL53L1X_ClearInterrupt(dev);

        // Log data over UART
        GPIO_PORTF_DATA_R |= 0x01;
        sprintf(printf_buffer, "%u, %d\r\n", Distance, angle);
        UART_printf(printf_buffer);
        SysTick_Wait10ms(20);
        GPIO_PORTF_DATA_R &= ~0x01;
      }

      // Rotate motor
      spin_degrees(spin_direction, 64);
      counter++;
      Buttons();
      if (!motor_running) break;
    }

    // Auto-stop after full rotation
    if(counter >= 32){
      motor_running = 0;
      acquisition_enabled = 0;
      GPIO_PORTF_DATA_R ^= 0x10;
    }
  }
}