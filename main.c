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
/*
 * Demo firmware reading LX34070A resolver signals on a dsPIC33CK.
 * The ADC collects sine and cosine, normalises them and computes
 * the angle via atan2f. All values are published through X2Cscope
 * so the Python GUIs can display them.
 *
 * The code also outputs quadrature A/B/Z pulses on RB7-9. With a
 * suitable count setting, the board becomes a resolver-to-encoder
 * converter that plugs into any controller expecting standard
 * incremental feedback.
 */

// ---------------------------------------------------------------------------
// Encoder GPIO helper macros (using RB7/RB8/RB9 for A/B/Z)
// ---------------------------------------------------------------------------
#ifndef ENCA_SetHigh
#define ENCA_SetHigh()   (_LATB7 = 1)
#define ENCA_SetLow()    (_LATB7 = 0)
#endif

#ifndef ENCB_SetHigh
#define ENCB_SetHigh()   (_LATB8 = 1)
#define ENCB_SetLow()    (_LATB8 = 0)
#endif

#ifndef ENCZ_SetHigh
#define ENCZ_SetHigh()   (_LATB9 = 1)
#define ENCZ_SetLow()    (_LATB9 = 0)
#endif

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
// Encoder emulation variables (ABZ)
static volatile uint8_t encoder_A = 0;
static volatile uint8_t encoder_B = 0;
static volatile uint8_t encoder_Z = 0;
static volatile uint16_t counts_per_rev = 500;     // counts per mechanical revolution
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

// Convert resolver_position to quadrature encoder signals
// Each mechanical revolution is split into counts_per_rev increments,
// producing standard A/B/Z pulses so external drives can treat the
// resolver like an incremental encoder.
static void UpdateEncoderOutputs(void)
{
    float angle = resolver_position;
    if(angle < 0.0f)
    {
        angle += 2.0f * (float)M_PI;
    }

    int totalStates = counts_per_rev * 4;
    int idx = (int)((angle / (2.0f * (float)M_PI)) * (float)totalStates);
    if(idx >= totalStates)
    {
        idx = totalStates - 1;
    }

    int quadState = idx % 4;
    int count = idx / 4;

    uint8_t a, b;
    switch(quadState)
    {
        case 0: a = 0; b = 0; break;
        case 1: a = 1; b = 0; break;
        case 2: a = 1; b = 1; break;
        default: a = 0; b = 1; break;
    }

    uint8_t z = (count == 0) ? 1 : 0;

    if(a != encoder_A)
    {
        encoder_A = a;
        if(a) ENCA_SetHigh(); else ENCA_SetLow();
    }
    if(b != encoder_B)
    {
        encoder_B = b;
        if(b) ENCB_SetHigh(); else ENCB_SetLow();
    }
    if(z != encoder_Z)
    {
        encoder_Z = z;
        if(z) ENCZ_SetHigh(); else ENCZ_SetLow();
    }
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
        if (sin_sampled && cos_sampled){ // to work correctly sin_calibrated and cos_calibrated need to be centered at zero
            resolver_position = atan2f(sin_calibrated, cos_calibrated); // returns value in radians [-π, π]
            UpdateEncoderOutputs();
            //resolver_sample_ready = true;
            // Reset for next sampling round
            sin_sampled = false;
            cos_sampled = false;
        }
        X2Cscope_Communicate();
    }    
}
