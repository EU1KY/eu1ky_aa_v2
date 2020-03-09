/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <complex.h> //C complex
#include <math.h>
#include "stm32f10x.h"
#include "dbgprint.h"
#include "osl.h"
#include "dsp.h"
#include "gen.h"
#include "config.h"
#include "lcd.h"
#include "font.h"
#include "touch.h"

//#pragma GCC diagnostic ignored "-Wunused-variable"
//#pragma GCC diagnostic ignored "-Wunused-function"

#define OSL_NSCANS  7
#define OSL_STEP    100000
#define OSL_STEPF   ((float)OSL_STEP)
#define R0 50.0f
#define OSL_NUM_ENTRIES (((BAND_FMAX) - (BAND_FMIN))/(OSL_STEP) + 1)

#define FLASH_PAGESIZE 2048


const float OSL_RLOAD = R0;
float OSL_RSHORT = 5.0f;   //can be changed via configuration menu
float OSL_ROPEN = 500.0f;  //can be changed via configuration menu

static const COMPLEX Z0 = R0 + 0.0fi;

static COMPLEX gammaShort;// = (OSL_RSHORT - R0)/(OSL_RSHORT + R0) + 0.0fi;
static const COMPLEX gammaLoad = 0.0 + 0.0fi;
static COMPLEX gammaOpen;// = (OSL_ROPEN - R0)/(OSL_ROPEN + R0) + 0.0fi;

static const COMPLEX cmplus1 = 1.0f + 0.0fi;
static const COMPLEX cmminus1 = -1.0f + 0.0fi;


static const int totalEntries = (int)OSL_NUM_ENTRIES;

extern void Sleep(uint32_t);

//Array for flash page data temporary buffer
static uint32_t flashTmpBuf[FLASH_PAGESIZE / sizeof(uint32_t)];

static int oslSelected = -1;

#pragma pack(push, 1)

//OSL calibration data structure
struct S_OSLDATA
{
    COMPLEX e00;    //e00 correction coefficient
    COMPLEX e11;    //e11 correction coefficient
    COMPLEX de;     //delta-e correction coefficient
    uint32_t state; //record state indicator: 0xFFFFFFFF for erased, 0 for valid
    uint32_t state2;
}; //32 bytes

static int StoreToFlash(struct S_OSLDATA* pData, uint32_t address);

//Temporary structure to store measurement data before calculating real correction coefficients
//Will be stored in place of S_OSLDATA
struct S_OSLTEMP
{
    COMPLEX gshort; //measured gamma for short load
    COMPLEX gload;  //measured gamma for 50 Ohm load
    COMPLEX gopen;  //measured gamma for open load
    uint32_t state; //record state indicator: 0xFFFFFFFF for erased
    uint32_t state2;
}; //32 bytes

#pragma pack(pop)

#define DIV_CEIL(x, y)        (((x)%(y) != 0) ? (x)/(y) + 1 : (x)/(y))
#define ALIGN_CEIL(x, y)      (DIV_CEIL((x), (y)) * (y))

#define OSL_NUM_ERRCORR_PAGES          DIV_CEIL((OSL_NUM_ENTRIES * sizeof(OSL_ERRCORR)), (FLASH_PAGESIZE))
#define OSLDATA_STORED_SIZE            (sizeof(struct S_OSLDATA) - 2 * sizeof(uint32_t))     //state vars are ignored
#define OSL_NUM_CAL_ENTRIES_PER_PAGE   ((FLASH_PAGESIZE)/sizeof(struct S_OSLDATA)) // 64
#define OSL_NUM_CAL_PAGES              DIV_CEIL((OSL_NUM_ENTRIES), (OSL_NUM_CAL_ENTRIES_PER_PAGE))

typedef struct { uint8_t bytes[FLASH_PAGESIZE]; } _STORAGE_PAGE;
#define ALLOC_STORAGE_PAGES(name, n) __attribute__ ((section ("flashstorage"))) _STORAGE_PAGE name[n]

ALLOC_STORAGE_PAGES(s_errCorr, OSL_NUM_ERRCORR_PAGES);
ALLOC_STORAGE_PAGES(s_oslData0, OSL_NUM_CAL_PAGES);
ALLOC_STORAGE_PAGES(s_oslData1, OSL_NUM_CAL_PAGES);
ALLOC_STORAGE_PAGES(s_oslData2, OSL_NUM_CAL_PAGES);
ALLOC_STORAGE_PAGES(s_TOUCH_cal, 1);

// Phase/Magnitude error correction array: phase offset
// and magnitude ratio coefficient for each frequency in BAND_FMIN ... BAND_FMAX range
// with 100 kHz steps (NERRCORR records)
// OSL_NUM_ERRCORR_PAGES pages
static const OSL_ERRCORR *errCorr = (const OSL_ERRCORR *)s_errCorr;

//Touchscreen calibration page (2k)
const struct TSCAL_PARAMS *TOUCH_cal = (const struct TSCAL_PARAMS *)s_TOUCH_cal;

//============================================================================

static const struct S_OSLDATA* oslFile[] =
{
    (const struct S_OSLDATA *)s_oslData0,
    (const struct S_OSLDATA *)s_oslData1,
    (const struct S_OSLDATA *)s_oslData2
};

//Called when loads changed in config menu
void OSL_RecalcLoads(void)
{
    gammaShort = (OSL_RSHORT - R0)/(OSL_RSHORT + R0) + 0.0fi;
    gammaOpen = (OSL_ROPEN - R0)/(OSL_ROPEN + R0) + 0.0fi;
}

void OSL_Select(int index)
{
    if (index >= 0 && index < (int)(sizeof(oslFile) / sizeof(void*)))
        oslSelected = index;
    else
        oslSelected = -1;
}

int OSL_GetSelected(void)
{
    if (oslSelected < 0 || oslSelected >= (int)(sizeof(oslFile) / sizeof(void*)))
        return -1;
    return oslSelected;
}

static int GetIndexForFreq(uint32_t fhz)
{
    int idx = -1;
    if (fhz >= BAND_FMIN && fhz <= BAND_FMAX)
    {
        idx = (int)roundf((float)fhz / OSL_STEPF) - BAND_FMIN/OSL_STEP;
        return idx;
    }
    return idx;
}

static uint32_t GetCalFreqByIdx(int idx)
{
    uint32_t oslCalFreqHz;
    if (idx < 0)
        return 0;
    oslCalFreqHz = BAND_FMIN + idx * OSL_STEP;

    if (oslCalFreqHz > BAND_FMAX)
        return 0;
    return oslCalFreqHz;
}

int OSL_CleanOSL(void)
{
    int i;
    if (OSL_GetSelected() < 0)
    {
        DBGPRINT("OSL_CleanOSL called without OSL file selected\n");
        return 0;
    }
    DBGPRINT("OSL_CleanOSL() for file %d\n", oslSelected);

    //Clear 16 pages in flash
    uint32_t pageaddr = (uint32_t)oslFile[oslSelected];
    FLASH_UnlockBank1();
    for (i = 0; i < OSL_NUM_CAL_PAGES; i++)
    {
        FLASH_Status status = FLASH_ErasePage(pageaddr);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("FLASH_ErasePage failed at 0x%08X, returned %d\n", (int)pageaddr, (int)status);
            return 0;
        }
        pageaddr += 0x800;
    }
    FLASH_LockBank1();
    return 1;
}

COMPLEX OSL_GFromZ(COMPLEX Z)
{
    COMPLEX G = (Z - Z0) / (Z + Z0);
    if (isnan(crealf(G)) || isnan(cimagf(G)))
    {
        DBGPRINT("OSL_GFromZ NaN\n");
        return 0.99999999f+0.0fi;
    }
    return G;
}

COMPLEX OSL_ZFromG(COMPLEX G)
{
    float gr2  = powf(crealf(G), 2);
    float gi2  = powf(cimagf(G), 2);
    float dg = powf((1.0f - crealf(G)), 2) + gi2;
    float r = R0 * (1.0f - gr2 - gi2) / dg;
    if (r < 0.0f) //Sometimes it overshoots a little due to limited calculation accuracy
        r = 0.0f;
    float x = R0 * 2.0f * cimagf(G) / dg;
    return r + x * I;
}

int OSL_ScanShort(void)
{
    if (OSL_GetSelected() < 0)
    {
        DBGPRINT("OSL_ScanShort called without OSL file selected\n");
        return 0;
    }

    struct S_OSLTEMP* ptr = (struct S_OSLTEMP*)oslFile[oslSelected];
    if ((ptr[0].state & 1) == 0)
    {
        DBGPRINT("OSL_ScanShort called for non-cleared file\n");
        return 0;
    }

    FLASH_UnlockBank1();
    int i;
    for (i = 0; ; i++)
    {
        uint32_t oslCalFreqHz = GetCalFreqByIdx(i);
        if (oslCalFreqHz == 0)
            break;
        if (i == 0)
            DSP_Measure(oslCalFreqHz, 1, 0, OSL_NSCANS); //First run is fake to let the filter stabilize
        FONT_ClearLine(FONT_FRANBIG, LCD_BLACK, 100);
        FONT_Print(FONT_FRANBIG, LCD_PURPLE, LCD_BLACK, 0, 100, "%d kHz", oslCalFreqHz/1000);

        DSP_Measure(oslCalFreqHz, 1, 0, OSL_NSCANS);

        DSP_RX rx = DSP_MeasuredZ();
        COMPLEX gamma = OSL_GFromZ(rx);

        uint32_t addr = (uint32_t)&ptr[i].gshort;
        uint32_t* pg = (uint32_t*)&gamma;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASH_Status status = FLASH_ProgramWord(addr, pg[0]);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanShort: FLASH_ProgramWord failed at 0x%08X, returned %d\n", (int)addr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
        addr += 4;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        status = FLASH_ProgramWord(addr, pg[1]);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanShort: FLASH_ProgramWord failed at 0x%08X, returned %d\n", (int)addr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
    }
    GEN_SetMeasurementFreq(0);
    //Write zero bit to indicate programmed state
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ProgramWord((uint32_t)&ptr[0].state, 0xFFFFFFFE);
    FLASH_LockBank1();
    return 1;
}

int OSL_ScanLoad(void)
{
    if (OSL_GetSelected() < 0)
    {
        DBGPRINT("OSL_ScanLoad called without OSL file selected\n");
        return 0;
    }

    struct S_OSLTEMP* ptr = (struct S_OSLTEMP*)oslFile[oslSelected];
    if ((ptr[0].state & 2) == 0)
    {
        DBGPRINT("OSL_ScanLoad called for non-cleared file\n");
        return 0;
    }

    FLASH_UnlockBank1();
    int i;
    for (i = 0; ; i++)
    {
        uint32_t oslCalFreqHz = GetCalFreqByIdx(i);
        if (oslCalFreqHz == 0)
            break;
        if (i == 0)
            DSP_Measure(oslCalFreqHz, 1, 0, OSL_NSCANS); //First run is fake to let the filter stabilize
        FONT_ClearLine(FONT_FRANBIG, LCD_BLACK, 100);
        FONT_Print(FONT_FRANBIG, LCD_PURPLE, LCD_BLACK, 0, 100, "%d kHz", oslCalFreqHz/1000);

        DSP_Measure(oslCalFreqHz, 1, 0, OSL_NSCANS);

        DSP_RX rx = DSP_MeasuredZ();
        COMPLEX gamma = OSL_GFromZ(rx);

        uint32_t addr = (uint32_t)&ptr[i].gload;
        uint32_t* pg = (uint32_t*)&gamma;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASH_Status status = FLASH_ProgramWord(addr, pg[0]);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanLoad: FLASH_ProgramWord failed at 0x%08X, returned %d\n", (int)addr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
        addr += 4;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        status = FLASH_ProgramWord(addr, pg[1]);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanLoad: FLASH_ProgramWord failed at 0x%08X, returned %d\n", (int)addr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
    }
    GEN_SetMeasurementFreq(0);
    //Write zero bit to indicate programmed state
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ProgramWord((uint32_t)&ptr[0].state, 0xFFFFFFFD);
    FLASH_LockBank1();
    return 1;
}

int OSL_ScanOpen(void)
{
    if (OSL_GetSelected() < 0)
    {
        DBGPRINT("OSL_ScanOpen called without OSL file selected\n");
        return 0;
    }

    struct S_OSLTEMP* ptr = (struct S_OSLTEMP*)oslFile[oslSelected];
    if ((ptr[0].state & 4) == 0)
    {
        DBGPRINT("OSL_ScanOpen called for non-cleared file\n");
        return 0;
    }

    FLASH_UnlockBank1();
    int i;
    for (i = 0; ; i++)
    {
        uint32_t oslCalFreqHz = GetCalFreqByIdx(i);
        if (oslCalFreqHz == 0)
            break;
        if (i == 0)
            DSP_Measure(oslCalFreqHz, 1, 0, OSL_NSCANS); //First run is fake to let the filter stabilize
        FONT_ClearLine(FONT_FRANBIG, LCD_BLACK, 100);
        FONT_Print(FONT_FRANBIG, LCD_PURPLE, LCD_BLACK, 0, 100, "%d kHz", oslCalFreqHz/1000);

        DSP_Measure(oslCalFreqHz, 1, 0, OSL_NSCANS);

        DSP_RX rx = DSP_MeasuredZ();
        COMPLEX gamma = OSL_GFromZ(rx);

        uint32_t addr = (uint32_t)&ptr[i].gopen;
        uint32_t* pg = (uint32_t*)&gamma;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASH_Status status = FLASH_ProgramWord(addr, pg[0]);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanOpen: FLASH_ProgramWord failed at 0x%08X, returned %d\n", (int)addr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
        addr += 4;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        status = FLASH_ProgramWord(addr, pg[1]);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanOpen: FLASH_ProgramWord failed at 0x%08X, returned %d\n", (int)addr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
    }
    GEN_SetMeasurementFreq(0);
    //Write zero bit to indicate programmed state
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ProgramWord((uint32_t)&ptr[0].state, 0xFFFFFFFB);
    FLASH_LockBank1();
    return 1;
}

// Function to calculate determinant of 3x3 matrix
// Input: 3x3 matrix [[a, b, c],
//                    [m, n, k],
//                    [u, v, w]]
static COMPLEX Determinant3(const COMPLEX a, const COMPLEX b, const COMPLEX c,
                            const COMPLEX m, const COMPLEX n, const COMPLEX k,
                            const COMPLEX u, const COMPLEX v, const COMPLEX w)
{
    return a * n * w + b * k * u + m * v * c - c * n * u - b * m * w - a * k * v;
}


//Cramer's rule implementation (see Wikipedia article)
//Solves three equations with three unknowns
static void CramersRule(const COMPLEX a11, const COMPLEX a12, const COMPLEX a13, const COMPLEX b1,
                        const COMPLEX a21, const COMPLEX a22, const COMPLEX a23, const COMPLEX b2,
                        const COMPLEX a31, const COMPLEX a32, const COMPLEX a33, const COMPLEX b3,
                        COMPLEX* pResult)
{
    COMPLEX div = Determinant3(a11, a12, a13, a21, a22, a23, a31, a32, a33);
    pResult[0] = Determinant3(b1, a12, a13, b2, a22, a23, b3, a32, a33) / div;
    pResult[1] = Determinant3(a11, b1, a13, a21, b2, a23, a31, b3, a33) / div;
    pResult[2] = Determinant3(a11, a12, b1, a21, a22, b2, a31, a32, b3) / div;
}

//Calculate OSL correction coefficients and write to flash in place of original values
int OSL_Calculate(void)
{
    int pgidx, recidx;
    uint32_t pgaddr;

    if (OSL_GetSelected() < 0)
    {
        DBGPRINT("OSL_Calculate called without OSL file selected\n");
        return 0;
    }

    {
        struct S_OSLTEMP* fptr = (struct S_OSLTEMP*)oslFile[oslSelected];
        pgaddr = (uint32_t)fptr;
        if ((fptr[0].state & 0x0F) != 0x08)
        {
            DBGPRINT("OSL_Calculate called for non-filled file. State = 0x%08X\n", (unsigned int)fptr[0].state);
            //return 0; //TODO! Why 0xFFFFFFE?
        }
    }

    int countEntries = 0;

    FLASH_UnlockBank1();
    //Handle OSL_NUM_CAL_PAGES pages one by one. OSL_NUM_CAL_ENTRIES_PER_PAGE OSL entries per page
    for (pgidx = 0; pgidx < OSL_NUM_CAL_PAGES; pgidx++)
    {
        //Copy page to buffer
        memcpy(flashTmpBuf, (void*)pgaddr, FLASH_PAGESIZE);
        struct S_OSLTEMP* fbuf = (struct S_OSLTEMP*)flashTmpBuf;

        //Erase page
        DBGPRINT("OSL_Calculate: Erasing page at 0x%08X\n", (unsigned int)pgaddr);
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASH_Status status = FLASH_ErasePage(pgaddr);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_Calculate: FLASH_ErasePage failed at 0x%08X, returned %d\n", (int)pgaddr, (int)status);
            FLASH_LockBank1();
            return 0;
        }

        //Calculate calibration coefficients and write to flash
        for (recidx = 0; recidx < OSL_NUM_CAL_ENTRIES_PER_PAGE; recidx++)
        {
            //DBGPRINT("OSL_Calculate: Record %d at page %d\n", recidx, pgidx);
            COMPLEX result[3]; //[e00, e11, de]
            COMPLEX a12 = gammaShort * fbuf[recidx].gshort;
            COMPLEX a22 = gammaLoad * fbuf[recidx].gload;
            COMPLEX a32 = gammaOpen * fbuf[recidx].gopen;
            COMPLEX a13 = cmminus1 * gammaShort;
            COMPLEX a23 = cmminus1 * gammaLoad;
            COMPLEX a33 = cmminus1 * gammaOpen;
            struct S_OSLDATA osldata;
            CramersRule( cmplus1, a12, a13, fbuf[recidx].gshort,
                         cmplus1, a22, a23, fbuf[recidx].gload,
                         cmplus1, a32, a33, fbuf[recidx].gopen,
                         result);
            osldata.e00 = result[0];
            osldata.e11 = result[1];
            osldata.de = result[2];
            osldata.state = 0xFFFFFFFF;
            if (!StoreToFlash(&osldata, pgaddr + recidx * sizeof(struct S_OSLDATA)))
            {
                FLASH_LockBank1();
                return 0;
            }
            countEntries++;
            if (countEntries >= totalEntries)
                break; //Early break at the last record
        }
        if (countEntries >= totalEntries)
            break; //Early break at the last record
        //Move to next page
        pgaddr += FLASH_PAGESIZE;
    }

    //Write 0xFFFFFFF0 to state word of the file to indicate calibration file is ready
    pgaddr = (uint32_t)&oslFile[oslSelected][0].state;
    DBGPRINT("OSL_Calculate: Writing status\n");
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_Status status = FLASH_ProgramWord(pgaddr, 0xFFFFFFF0);
    FLASH_LockBank1();
    if (status != FLASH_COMPLETE)
    {
        DBGPRINT("OSL_Calculate: FLASH_ProgramWord failed at 0x%08X, returned %d\n",
                 (int)pgaddr, (int)status);
        return 0;
    }
    return 1;
}

static int StoreToFlash(struct S_OSLDATA *pOslData, uint32_t address)
{
    uint32_t* pData = (uint32_t*)pOslData;
    size_t i;
    FLASH_UnlockBank1();
    for (i = 0; i < OSLDATA_STORED_SIZE; i += 4) //Don't flash state words!
    {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASH_Status status = FLASH_ProgramWord(address + i, *pData++);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("StoreToFlash: FLASH_ProgramWord failed at 0x%08X, returned %d\n",
                     (int)(address+i), (int)status);
            FLASH_LockBank1();
            return 0;
        }
    }
    return 1;
}

int OSL_ScanADCCorrection(void)
{
    int i;
    FLASH_Status status;

    //Erase 6 pages where array is stored
    FLASH_UnlockBank1();
    // Clear all pending flags
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    for (i = 0; i < OSL_NUM_ERRCORR_PAGES; i++)
    {
        // Erase the FLASH page
        status = FLASH_ErasePage((uint32_t)errCorr + i * FLASH_PAGESIZE);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("FLASH_ErasePage failed at OSL_ScanADCCorrection, returned %d\n", (int)status);
            FLASH_LockBank1();
            return 0;
        }
    }

    //Scan and store corrections
    uint32_t fhz;
    for (fhz = BAND_FMIN; fhz <= BAND_FMAX; fhz += OSL_STEP)
    {
        uint32_t pgaddr;
        uint32_t* pVal;
        int i = GetIndexForFreq(fhz);

        FONT_ClearLine(FONT_FRANBIG, LCD_BLACK, 100);
        FONT_Print(FONT_FRANBIG, LCD_GREEN, LCD_BLACK, 0, 100, "%d of %d (%d kHz)", i, totalEntries, ((int)fhz)/1000);

        DSP_Measure(fhz, 0, 0, OSL_NSCANS); //Perform measurement without HW error correction

        OSL_ERRCORR ec;
        ec.mag0 = 1.0 / DSP_MeasuredDiff();
        ec.phase0 = DSP_MeasuredPhase();

        pgaddr = (uint32_t)(errCorr) + i * sizeof(OSL_ERRCORR);

        pVal = (uint32_t*)&ec.mag0;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        status = FLASH_ProgramWord(pgaddr, *pVal);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanADCCorrection: FLASH_ProgramWord failed at 0x%08X, returned %d\n",
                     (int)pgaddr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
        pgaddr += 4;
        pVal = (uint32_t*)&ec.phase0;
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        status = FLASH_ProgramWord(pgaddr, *pVal);
        if (status != FLASH_COMPLETE)
        {
            DBGPRINT("OSL_ScanADCCorrection: FLASH_ProgramWord failed at 0x%08X, returned %d\n",
                     (int)pgaddr, (int)status);
            FLASH_LockBank1();
            return 0;
        }
    }
    GEN_SetMeasurementFreq(0);
    FLASH_LockBank1();
    return 1;
}

//Get HW error correction
OSL_ERRCORR OSL_GetADCCorrection(uint32_t fhz)
{
    OSL_ERRCORR c = {1.0, 0.0};
    int idx = GetIndexForFreq(fhz);
    if (idx == -1)
        return c;
    c = errCorr[idx]; //Get record from array
    if (isnan(c.mag0) || isinf(c.mag0) || isnan(c.phase0) || isinf(c.phase0))
    {//Bad record read from flash
        c.mag0 = 1.0;
        c.phase0 = 0.0;
    }
    return c;
}



//Parabolic interpolation
// Let (x1,y1), (x2,y2), and (x3,y3) be the three "nearest" points and (x,y)
// be the "concerned" point, with x2 < x < x3. If (x,y) is to lie on the
// parabola through the three points, you can express y as a quadratic function
// of x in the form:
//    y = a*(x-x2)^2 + b*(x-x2) + y2
// where a and b are:
//    a = ((y3-y2)/(x3-x2)-(y2-y1)/(x2-x1))/(x3-x1)
//    b = ((y3-y2)/(x3-x2)*(x2-x1)+(y2-y1)/(x2-x1)*(x3-x2))/(x3-x1)
static COMPLEX ParabolicInterpolation(COMPLEX y1, COMPLEX y2, COMPLEX y3, //values for frequencies x1, x2, x3
                               float x1, float x2, float x3,       //frequencies of respective y values
                               float x) //Frequency between x2 and x3 where we want to interpolate result
{
    COMPLEX a = ((y3-y2)/(x3-x2)-(y2-y1)/(x2-x1))/(x3-x1);
    COMPLEX b = ((y3-y2)/(x3-x2)*(x2-x1)+(y2-y1)/(x2-x1)*(x3-x2))/(x3-x1);
    COMPLEX res = a * powf(x - x2, 2.) + b * (x - x2) + y2;
    return res;
}

//Correct measured G using selected OSL calibration file
COMPLEX OSL_CorrectG(uint32_t fhz, COMPLEX gMeasured)
{
    if (fhz < BAND_FMIN || fhz > BAND_FMAX) //We can't do anything with frequencies beyond the range
    {
        DBGPRINT("OSL_CorrectG called with out-of-band freq %d\n", (int)fhz);
        return gMeasured;
    }
    if (OSL_GetSelected() < 0)
    {
        DBGPRINT("OSL_CorrectG called without OSL file selected\n");
        return gMeasured;
    }

    struct S_OSLDATA* ptr = (struct S_OSLDATA*)oslFile[oslSelected];
    if ((ptr[0].state) != 0xFFFFFFF0)
    {
        DBGPRINT("OSL_CorrectG called for invalid OSL file\n");
        return gMeasured;
    }

    int i;
    struct S_OSLDATA oslData = {0};
    i = (fhz - BAND_FMIN) / OSL_STEP; //Nearest lower OSL file record index

    if ((fhz % OSL_STEP) == 0) //We already have exact value for this frequency
        oslData = ptr[i];
    else if (i == 0)
    {//Linearly interpolate two OSL factors for two nearby records
     //(there is no third point for this interval)
        float prop = (fhz - BAND_FMIN) / OSL_STEPF; //proportion
        oslData.e00 = (ptr[i+1].e00 - ptr[i].e00) * prop + ptr[i].e00;
        oslData.e11 = (ptr[i+1].e11 - ptr[i].e11) * prop + ptr[i].e11;
        oslData.de = (ptr[i+1].de - ptr[i].de) * prop + ptr[i].de;
    }
    else
    {//We have three OSL points near fhz, thus using parabolic interpolation
        float f1, f2, f3;
        f1 = (i - 1) * OSL_STEP + (float)(BAND_FMIN);
        f2 = i * OSL_STEPF + (float)(BAND_FMIN);
        f3 = (i + 1) * OSL_STEPF + (float)(BAND_FMIN);
        oslData.e00 = ParabolicInterpolation(ptr[i-1].e00, ptr[i].e00, ptr[i+1].e00,
                                             f1, f2, f3, (float)(fhz));
        oslData.e11 = ParabolicInterpolation(ptr[i-1].e11, ptr[i].e11, ptr[i+1].e11,
                                             f1, f2, f3, (float)(fhz));
        oslData.de = ParabolicInterpolation(ptr[i-1].de, ptr[i].de, ptr[i+1].de,
                                             f1, f2, f3, (float)(fhz));

    }
    //At this point oslData contains correction structutre for given frequency fhz
    COMPLEX gResult = (gMeasured - oslData.e00) / (gMeasured * oslData.e11 - oslData.de);
    DBGPRINT("For F=%d Hz corrected G = %.3f %+.3fj\n", (int)fhz, crealf(gResult), cimagf(gResult));
    return gResult;
}

COMPLEX OSL_CorrectZ(uint32_t fhz, COMPLEX zMeasured)
{
    COMPLEX g = OSL_GFromZ(zMeasured);
    g = OSL_CorrectG(fhz, g);
    if (crealf(g) > 1.0f)
        g = 1.0f + cimagf(g)*I;
    else if (crealf(g) < -1.0f)
        g = -1.0f + cimagf(g)*I;
    if (cimagf(g) > 1.0f)
        g = crealf(g) + 1.0f * I;
    else if (cimagf(g) < -1.0f)
        g = crealf(g) - 1.0f * I;
    g = OSL_ZFromG(g);
    DBGPRINT("For F=%d Hz corrected Z = %.3f %+.3fj\n", (int)fhz, crealf(g), cimagf(g));
    return g;
}
