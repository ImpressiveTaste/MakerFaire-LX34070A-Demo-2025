/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/X2Cscope/X2Cscope.h"
#include "mcc_generated_files/system/system.h"

/*
    Main application
*/

#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/adc/adc1.h" // TODO: Replace {adc_header} with the corresponding ADC header file for your project (ex: adc1.h)
#include "mcc_generated_files/timer/sccp1.h" // TODO: Replace {timer_header} with the corresponding timer header file for your project (ex: tmr1.h)
#include "mcc_generated_files/timer/sccp2.h" // TODO: Replace {timer_header} with the corresponding timer header file for your project (ex: tmr1.h)

#include <math.h> // For atan2f

static struct ADC_INTERFACE *adc = &ADC1; // TODO: Replace with the ADC instance in your project

static volatile uint16_t sin_raw=0;
static volatile uint16_t cos_raw=0;

static volatile float sin_offset=1350f;
static volatile float cos_offset=1350f;

static volatile float sin_amplitude = 7.999999797903001E-4;      // Gain to normalize amplitude
static volatile float cos_amplitude = 7.999999797903001E-4;
static volatile float sin_calibrated = 0.0f;
static volatile float cos_calibrated = 0.0f;
//static bool printResult = false;
static volatile float resolver_position = 0.0f;   // In radians
//static volatile bool resolver_sample_ready = false;
// Internal flags to track sampling state
static bool sin_sampled = false;
static bool cos_sampled = false;

static void ADC_ConversionDone(enum ADC_CHANNEL channel, uint16_t adcVal)
{
    if(channel == Channel_AN0) // TODO: Replace with the ADC Channel you selected in the Pin Grid View 
    {
        sin_raw = adcVal; 
        sin_calibrated = ((float)adcVal - sin_offset) * sin_amplitude;     
        sin_sampled = true;
        //printResult = true;
    }
    else if(channel == Channel_AN1) // TODO: Replace with the ADC Channel you selected in the Pin Grid View 
    {
        cos_raw = adcVal; 
        cos_calibrated = ((float)adcVal - cos_offset) * cos_amplitude;        
        cos_sampled = true;
        //printResult = true;
    }
}

static void ADC_StartConversionOnChannel(void)
{
    adc->SoftwareTriggerEnable();
}

static void Blink_LED(void)
{
    LED0_Toggle();
}

int main(void)
{
    
    const struct TIMER_INTERFACE *timer1 = &Timer1; // TODO: Replace with the timer instance in your project
    const struct TIMER_INTERFACE *timer2 = &Timer2; // TODO: Replace with the timer instance in your project

    SYSTEM_Initialize();

    adc->adcMulticoreInterface->ChannelCallbackRegister(ADC_ConversionDone);
    timer1->TimeoutCallbackRegister(ADC_StartConversionOnChannel);
    timer2->TimeoutCallbackRegister(Blink_LED);

    while(1)
    {
        if (sin_sampled && cos_sampled){//to qork correctly sin_calibrated and cos_calibrated need to becenterd at zero
        resolver_position = atan2f(sin_calibrated, cos_calibrated); // returns value in radians [-π, π]
        //resolver_sample_ready = true;
        // Reset for next sampling round
        sin_sampled = false;
        cos_sampled = false;
        }
        X2Cscope_Communicate();
    }    
}
