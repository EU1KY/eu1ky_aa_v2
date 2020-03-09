/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <string.h>
#include <math.h>
#include <limits.h>
#include <complex.h>
#include "stm32f10x.h"
#include "dsp.h"
#include "dbgprint.h"
#include "gen.h"
#include "osl.h"
#include "config.h"

#define Rmeas 10.0f
#define RmeasAdd 33.0f
#define Rload 51.0f
#define Rtotal (RmeasAdd + Rmeas + Rload)
#define DSP_Z0 50.0f

#define NSAMPLES 512

#define DSP_SAMPLECYCLES ADC_SampleTime_1Cycles5

extern void Sleep(uint32_t);

int g_favor_precision = 0; //Nonzero value increases number of retries in case of noise detection during calibration

//Considering ADC clock 9 MHz, 1.5 ADC clocks for sampling, and 12.5 clocks for
//conversion, we obtain sampling frequency for our DSP algorithms:
//static const float fSample = (9000.f / 14.f) * 1000.f;
static const float targetFreq = 10045.f; //Falls exactly in the middle of bin

//Goertzel algorithm constants
#define BIN 8 //(int)roundf(((0.5f + NSAMPLES) * targetFreq) / fSample);
#define GOERTZEL_W ((2. * M_PI / NSAMPLES) * BIN)
static float cosine = 0.;
static float sine = 0.;
static float coeff = 0.;

//Magnitude conversion coefficient (to millivolts):
//2.34 is amplitude loss because of the Blackman window
//3.3 is power supply voltage (==Aref)
//4096 is ADC resolution (12 bits)
static const float MCF = (2.34 * 3.3 * 2.0 * 1000.) / (NSAMPLES * 4096);

//Sample buffer (filled with DMA)
static volatile uint32_t adcBuf[NSAMPLES];
static float windowfunc[NSAMPLES];

//Measurement results
static float complex magphase_i = 0.1f+0.fi; //Measured magnitude and phase for I channel
static float complex magphase_q = 0.1f+0.fi; //Measured magnitude and phase for Q channel
static float magmv_i = 1.;                   //Measured magnitude in millivolts for I channel
static float magmv_q = 1.;                   //Measured magnitude in millivolts for I channel
static float magdif = 1.f;                   //Measured magnitude ratio
static float magdifdb = 0.f;                 //Measured magnitude ratio in dB
static float phdif = 0.f;                    //Measured phase difference in radians
static float phdifdeg = 0.f;                 //Measured phase difference in degrees
static DSP_RX mZ = DSP_Z0 + 0.0fi;

static float DSP_CalcR(void);
static float DSP_CalcX(void);

static float complex Goertzel(int channel)
{
    int i;
    float q0, q1, q2;
    float re, im, magnitude, phase;

    //initials
    q0 = 0.0;
    q1 = 0.0;
    q2 = 0.0;
    for (i = 0; i < NSAMPLES; i++)
    {
        float sample = (channel ? (float)(adcBuf[i] >> 16) : (float)(adcBuf[i] & 0xFFFF));
        //Remove DC component as much as possible
        sample -= 2048.0;
        //Apply windowing to reduce reaction of this bin on OOB frequencies
        sample *= windowfunc[i];

        q0 = coeff * q1 - q2 + sample;
        q2 = q1;
        q1 = q0;
    }

    //Run one extra step with sample = 0 to obtain exactly the same result as in DFT
    //for particular bin (see here: http://en.wikipedia.org/wiki/Goertzel_algorithm)
    q0 = coeff * q1 - q2; //considering sample is 0
    q2 = q1;
    q1 = q0;

    //Calculate results
    re = q1 - q2 * cosine;
    im = q2 * sine;
    magnitude = sqrtf(powf(re, 2) + powf(im, 2));
    phase = atan2f(im, re);
    return magnitude + phase * I;
}


void ConfigDMA(void)
{
    DMA_InitTypeDef   DMA_InitStructure;
    //DMA1 for ADC1
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = NSAMPLES;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

//Prepare ADC for sampling two channels
void DSP_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_InitTypeDef   ADC_InitStructure;
    int i;

    // PCLK2 is the APB2 clock
    // ADCCLK = PCLK2/6 = 72/6 = 12MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); //1.5 MHz

    //Enable DMA clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // Enable GPIOC, ADC, TIM1 and AFIO clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
                           RCC_APB2Periph_TIM1, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ConfigDMA();

    //Enable ADC1 on PC0, ADC3 on PC1
    //(ADC Channel10, ADC Channel11)
    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, DSP_SAMPLECYCLES);
    ADC_DMACmd(ADC1, ENABLE);

    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, DSP_SAMPLECYCLES);
    ADC_ExternalTrigConvCmd(ADC2, ENABLE);

    Sleep(10);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_Cmd(ADC2, ENABLE);
    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));

    //Generate window function to minimize bin sensitivity to out-of-band noise
    for (i = 0; i < NSAMPLES; i++)
    {
        //Hamming window, 43 dB OOB rejection
        //windowfunc[i] = 0.54f - 0.46f * cosf((2 * M_PI * i) / NSAMPLES);

        //Blackman window, >66 dB OOB rejection
        windowfunc[i] = 0.426591f - .496561f * cosf( (2 * M_PI * i) / NSAMPLES) + .076848f * cosf((4 * M_PI * i) / NSAMPLES);
    }

    //Goertzel constants
    cosine = cosf(GOERTZEL_W);
    sine = sinf(GOERTZEL_W);
    coeff = 2. * cosine;

    DBGPRINT("DSP initialized\n");
}

#define MAXNMEAS 50
static float mag_i_buf[MAXNMEAS];
static float mag_q_buf[MAXNMEAS];
static float phdif_buf[MAXNMEAS];

//Filter array of floats with nm entries to remove outliers, and return mean
//of the remaining entries that fall into 1 sigma interval.
//In normal distribution, which is our case, 68% of entries fall into single
//standard deviation range.
static float DSP_FilterArray(float* arr, int nm, int doRetries)
{
    int i;
    int counter;
    float result;
    float low;
    float high;
    float deviation;
    float mean;

    if (nm <= 0)
        return 0.0f;
    else if (nm > MAXNMEAS)
        nm = MAXNMEAS;

    //Calculate mean
    mean = 0.0f;
    for (i = 0; i < nm; i++)
        mean += arr[i];
    mean /= nm;

    if (nm < 5)
    {//Simple case. Just return mean.
        return mean;
    }
    //============================
    // Filtering outliers
    //============================

    //Calculate standard deviation (sigma)
    deviation = 0.0f;
    for (i = 0; i < nm; i++)
    {
        float t = arr[i] - mean;
        t  = t * t;
        deviation += t;
    }
    deviation = sqrtf(deviation / nm);

    //Calculate mean of entries within part of standard deviation range
#ifdef FAVOR_PRECISION
    if (g_favor_precision)
    {
        low = mean - deviation * 0.6;
        high = mean + deviation * 0.6;
    }
    else
#endif
    {
        low = mean - deviation * 0.75;
        high = mean + deviation * 0.75;
    }
    counter = 0;
    result = 0.0f;
    for (i = 0; i < nm; i++)
    {
        if (arr[i] >= low && arr[i] <= high)
        {
            result += arr[i];
            counter++;
        }
    }
    if (doRetries && counter < nm/2)
    {
        return 0.0;
    }
    if (counter == 0)
    {//Oops! Nothing falls into the range, so let's simply return mean
        return mean;
    }
    result /= counter;
    return result;
}

//Set frequency, run measurement sampling and calculate phase, magnitude ratio
//and Z from sampled data, applying hardware error correction and OSL correction
//if requested. Note that clock source remains turned on after the measurement!
void DSP_Measure(uint32_t freqHz, int applyErrCorrection, int applyOSL, int nMeasurements)
{
    #define NMEAS nMeasurements
    float mag_i = 0.0f;
    float mag_q = 0.0f;
    float pdif = 0.0f;
    float complex res_i, res_q;
    int i;
    #ifdef FAVOR_PRECISION
    int retries = g_favor_precision ? 150 : 10;
    #else
    int retries = 3;
    #endif
    OSL_ERRCORR errCorr;

    assert_param(nMeasurements > 0);
    if (nMeasurements > MAXNMEAS)
        nMeasurements = MAXNMEAS;

    if (freqHz != 0)
    {
        GEN_SetMeasurementFreq(freqHz);
    }
    freqHz = GEN_GetLastFreq();

    DBGPRINT("DSP: measuring at %d Hz\n", (int)freqHz);
    if (applyErrCorrection)
    {
        errCorr = OSL_GetADCCorrection(freqHz);
    }
REMEASURE:
    for (i = 0; i < NMEAS; i++)
    {
        ConfigDMA();
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(!DMA_GetFlagStatus(DMA1_FLAG_TC1)); //Blocks until measurement is finished
        DMA_ClearFlag(DMA1_FLAG_TC1);
        ADC_SoftwareStartConvCmd(ADC1, DISABLE);

        //Apply Goertzel agorithm to sampled data
        res_i = Goertzel(0);
        res_q = Goertzel(1);

        mag_i_buf[i] = crealf(res_i);
        mag_q_buf[i] = crealf(res_q);
        pdif = cimagf(res_i) - cimagf(res_q);
        //Correct phase difference quadrant
        pdif = fmodf(pdif + M_PI, 2 * M_PI) - M_PI;

        if (pdif < -M_PI)
            pdif += 2 * M_PI;
        else if (pdif > M_PI)
            pdif -= 2 * M_PI;
#ifdef F_LO_DIVIDED_BY_TWO
        //Correct quadrature phase shift
        if (freqHz > BAND_FMAX) //Working on 3rd harmonic of LO
            pdif -= M_PI_2;
        else
            pdif += M_PI_2;
#endif
        phdif_buf[i] = pdif;
    }

    //Now perform filtering to remove outliers with sigma > 1.0
    mag_i = DSP_FilterArray(mag_i_buf, NMEAS, retries);
    mag_q = DSP_FilterArray(mag_q_buf, NMEAS, retries);
    phdif = DSP_FilterArray(phdif_buf, NMEAS, retries);
    if (mag_i == 0.0f || mag_q == 0.0f || phdif == 0.0f)
    {//need to measure again : too much noise detected
        retries--;
        goto REMEASURE;
    }

    if (applyErrCorrection)
    {
        mag_q *= errCorr.mag0;
        phdif  -= errCorr.phase0;
    }

    //Calculate derived results
    magmv_i = mag_i * MCF;
    magmv_q = mag_q * MCF;

    magdif = mag_q / mag_i;
    phdifdeg = (phdif * 180.) / M_PI;
    magdifdb = 20 * log10f(magdif);
    mZ = DSP_CalcR() + DSP_CalcX() * I;
    DBGPRINT("For f=%u before OSL Z=%.1f%+.1fj\n", (unsigned int)freqHz, crealf(mZ), cimagf(mZ));

    //Apply OSL correction if needed
    if (applyOSL)
    {
        mZ = OSL_CorrectZ(freqHz, mZ);
    }
    DBGPRINT("For f=%u measured Z=%.1f%+.1fj\n", (unsigned int)freqHz, crealf(mZ), cimagf(mZ));
}

//Return last measured Z
DSP_RX DSP_MeasuredZ(void)
{
    return mZ;
}

//Return last measured phase shift
float DSP_MeasuredPhase(void)
{
    return phdif;
}

float DSP_MeasuredPhaseDeg(void)
{
    return phdifdeg;
}

float DSP_MeasuredDiffdB(void)
{
    return magdifdb;
}

float DSP_MeasuredDiff(void)
{
    return magdif;
}

float complex DSP_MeasuredMagPhaseI(void)
{
    return magphase_i;
}

float complex DSP_MeasuredMagPhaseQ(void)
{
    return magphase_q;
}

float DSP_MeasuredMagImv(void)
{
    return magmv_i;
}

float DSP_MeasuredMagQmv(void)
{
    return magmv_q;
}

static float DSP_CalcR(void)
{
    float RR = (cosf(phdif) * Rtotal * magdif) - (Rmeas + RmeasAdd);
    if(RR < 0.0f) //Sometimes this happens due to measurement inaccuracy
        RR = 0.0f;
    return RR;
}

static float DSP_CalcX(void)
{
    return sinf(phdif) * Rtotal * magdif;
}


//Calculate VSWR from Z
float DSP_CalcVSWR(DSP_RX Z)
{
    float X2 = powf(cimagf(Z), 2);
    float R = crealf(Z);
    if(R < 0.0)
    {
        R = 0.0;
    }
    float ro = sqrtf((powf((R - DSP_Z0), 2) + X2) / (powf((R + DSP_Z0), 2) + X2));
    if(ro > .999)
    {
        ro = 0.999;
    }
    X2 = (1.0 + ro) / (1.0 - ro);
    DBGPRINT("VSWR %.2f\n", X2);
    return X2;
}

uint32_t DSP_GetIF(void)
{
    return (uint32_t)targetFreq;
}
