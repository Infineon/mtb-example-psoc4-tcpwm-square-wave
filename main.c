/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PSoC 4 TCPWM Square Wave
* code example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * System entrance point. This function performs
 *   1. Initializes the BSP.
 *   2. Calls the functions to set up PWM peripheral.
 *   3. Put the CPU to sleep mode.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize, enable, and start the PWM component.
     * To see all pre-defined parameters such as <Instance_Name>_Mask/Num
     * or PWM_config see cycfg_peripherals.h under generated sources.
     */
    (void) Cy_TCPWM_PWM_Init(PWM_HW, PWM_NUM, &PWM_config);
    Cy_TCPWM_Enable_Multiple(PWM_HW, PWM_MASK);
    Cy_TCPWM_TriggerReloadOrIndex(PWM_HW, PWM_MASK);

    /* Infinite loop */
    for(;;)
    {
        /* Put the CPU into sleep mode to save power */
        Cy_SysPm_CpuEnterSleep();
    }
}

/* [] END OF FILE */
