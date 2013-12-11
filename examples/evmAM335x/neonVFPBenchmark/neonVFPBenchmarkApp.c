/**
 * \file   neonVFPBenchmarkApp.c
 *
 * \brief  This is a sample application which measures the
 *         execution time of functions involving vectorized float
 *         operations using the SIMD capable Neon CoProcessor or
 *         floating point VFP Coprocessor.
 *
 *         Application Configuration:
 *
 *             Modules Used:
 *                 DMTimer7
 *                 UART0
 *
 *             Configurable parameters:
 *                 None.
 *
 *         Application Use Case:
 *             1: The application demonstrates basic benchmarking of
 *                functions/routines for testing the performance of
 *                on SIMD capable Neon Coprocessor and floating point
 *                VFP CoProcessor.
 *             2: The application demonstrates performance measurement
 *                of Floating point Additions and Multiplications for
 *                all ToolChains supported(GCC, CCS and IAR).
 *             3: Additionally for GCC compiler the application measures
 *                performance of Sine and Cosine Maths functions for
 *                Neon engine, implemented using with and without
 *                Neon Intrinsics.
 *
 *         Running the example:
 *             1: On execution, the example will measure the time taken by
 *                Function plugged into the Benchmark structure defined
 *                and display the result on the console.
 *             2: By default the example will give performance numbers
 *                of the functions executing on Neon engine.
 *             3: To get the VFP and SoftFloat numbers(Library support
 *                for floating point operations) the user has to
 *                recompile the example with VFP and no Coprocessor
 *                support option respectively.
 *             4: Refer to the User Guide for steps and
 *                different configurations to generate the binaries with
 *                and without the CoProcessor Option.
 *
 *         Limitations:
 *             1: The system timer used to measure the performance places
 *                a limitation of 170 seconds for overflow condition.
 *             2: Any routine which takes more time than 170s overflow limit
 *                cannot be measured accurately.
 *             3: For CCS and IAR Toolchain simple Functions demonstrating
 *                float operations on arrays are used to demonstrate
 *                benchmarking.
 *             4: The Intrinsics function used for GCC are not compatible with
 *                IAR and CCS compiler.
 *
 */

/*
* Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#include "evmAM335x.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "consoleUtils.h"
#include "hw_types.h"
#include <string.h>
#include "cache.h"
#include "mmu.h"
#include "perf.h"
#include "clock.h"
#if defined(gcc) && defined(__ARM_NEON__)
#include "port_mathLib.h"
#endif

/*
** Structure object declaration which includes the function
** to be benchmarked, the number of iterations to run
** and the name of the function.
*/
 struct benchmarkFunction
{
   void (*benchmarkRoutine) (void);
   unsigned int iterations;
   const char *name;
};

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR                  (0x80000000u)
#define START_ADDR_DEV                  (0x44000000u)
#define START_ADDR_OCMC                 (0x40300000u)
#define NUM_SECTIONS_DDR                (512u)
#define NUM_SECTIONS_DEV                (960u)
#define NUM_SECTIONS_OCMC               (1u)

#define CONST_PI_VAL                    (3.142)
#define VECTOR_SIZE                     (200u)
#define TIME_GRANULARITY                (1000000u)

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

static float ResultVector[200], VectorA[200], VectorB[200];

/* Page tables start must be aligned to 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(PageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int PageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int PageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(gcc)
static volatile unsigned int PageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

#else
#error "Unsupported Compiler.\r\n"

#endif
/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/
static void _MMUConfigAndEnable(void);
static void _NeonVFPBenchmarkRun(struct benchmarkFunction *benchMarkFxnInfo);
static void _BenchmarkVectorScaleFn();
static void _BenchmarkVectorProductFn();
static void _VectorFloatScale(float *vectorA, float *vectorB, float * result);
static void _VectorFloatMultiply(float *vectorA, float *vectorB, float *result);
static void _BenchmarkVectorArrayInit();

/*
** Initialize the array with the Pointers to the Routines whose
** performance have to be benchmarked.
*/
struct benchmarkFunction PluginObject[] =
{
    {_BenchmarkVectorScaleFn, 100000, "VectorScale"},
    {_BenchmarkVectorProductFn, 100000, "VectorProduct"},
#if defined(gcc) && defined(__ARM_NEON__)
    {BenchmarkCephesSine, 100000, "CephesLibSine"},
    {BenchmarkCephesCosine, 100000, "CephesLibCosine"},
    {BenchmarkIntrinsicSine, 100000, "IntrinsicSine"},
    {BenchmarkIntrinsicCosine, 100000, "IntrinsicCosine"},
#endif
};

/****************************************************************************/
/*                   FUNCTION DEFINITIONS                             */
/****************************************************************************/

/*
** Main function.
*/
int main(void)
{
    unsigned int index = 0u;
    unsigned int count = 0u;

    /* Configure and enable the MMU. */
    _MMUConfigAndEnable();

    /* Enable all levels of Cache. */
    CacheEnable(CACHE_ALL);

    /* Initialize the Console */
    ConsoleUtilsInit();

    /*
    ** Select the console type where you want to redirect all
    ** console IO operations.
    */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Initialize the Maths library Intrinsic functions for Benchmarking */
    #if defined(gcc) && defined(__ARM_NEON__)
    CephesMathLibInit();
    #endif

    /* Initialize the floating point arrays */
    _BenchmarkVectorArrayInit();

    ConsoleUtilsPrintf("**** Starting Benchmarking of Functions *****\r\n");

    /* Setup the Timer for Performance measurement */
    PerfTimerSetup();

    count = (sizeof(PluginObject) / sizeof(PluginObject[0]));

    for(index = 0; index < count; index++)
    {
        _NeonVFPBenchmarkRun(PluginObject + index);
    }

    ConsoleUtilsPrintf("\r\n**** Benchmarking of the functions done. ****\r\n");

    while(1);
}

/*
** Function to setup MMU. This function Maps three regions ( 1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void _MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes
    ** specified here are,
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)PageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)PageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)PageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)PageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)PageTable);
}

/*
** Generic Wrapper function which invokes any function whose
** Performance is to be measured. The Benchmarking Routine
** assigned to the benchMark Object, is run and the
** corresponding time taken by the function is measured.
** The Performance result is printed on the console.
*/
static void _NeonVFPBenchmarkRun(struct benchmarkFunction *benchMarkFxnInfo)
{
    unsigned int timerTicks = 0u;
    unsigned int count = 0u;
    unsigned int time = 0u;

    /* Start the Timer for Performance Measurement */
    PerfTimerStart();

    /*
    ** Run the Benchmarking Routine for specified iterations
    ** to get the Performance metric for the function
    */
    for(count = 0; count < (benchMarkFxnInfo->iterations); count++)
    {
        benchMarkFxnInfo->benchmarkRoutine();
    }

    /* Stop the Timer and read the Ticks */
    timerTicks = PerfTimerStop();

    /* Get the time taken in from the ticks read from the Timer */
    time = (timerTicks/ (CLK_EXT_CRYSTAL_SPEED/TIME_GRANULARITY));

    ConsoleUtilsPrintf("\r\n%s function takes %u microseconds "
                       "to run %u times\r\n",
                       benchMarkFxnInfo->name, time,
                       benchMarkFxnInfo->iterations);

    ConsoleUtilsPrintf("The ticks clocked by the timer is %u\r\n",timerTicks);
}

/* This function initializes the floating point array with float values */
static void _BenchmarkVectorArrayInit()
{
    unsigned int index = 0u;

    for(index = 0; index < VECTOR_SIZE; index++)
    {
        VectorA[index] = 1.0221 * index;
        VectorB[index] = 2.127 * index;
        ResultVector[index] = 0;
    }
}

/* Wrapper function which calls Vector Product function. */
static void _BenchmarkVectorProductFn()
{
    _VectorFloatMultiply(VectorA, VectorB, ResultVector);
}

/* Wrapper function which calls Vector Add and Scale function. */
static void _BenchmarkVectorScaleFn()
{
    _VectorFloatScale(VectorA, VectorB, ResultVector);
}

/*
** This function multiplies two single precision float arrays of
** and stores the result in another array.
*/
static void _VectorFloatMultiply(float *vectorA, float *vectorB, float *result)
{
    unsigned int index = 0u;

    for(index = 0; index < VECTOR_SIZE; index++)
    {
        result[index] = vectorA[index] * vectorB[index];
    }
}

/*
** This function executes the following operation
** C[i] = k * A[i] + B[i] on 2 single precision float point arrays
** and stores the result in another float array.
*/
static void _VectorFloatScale(float *vectorA, float *vectorB, float *result)
{
    unsigned int index = 0u;

    for(index = 0; index < VECTOR_SIZE; index++)
    {
        result[index] = (vectorA[index] * CONST_PI_VAL) + vectorB[index];
    }
}
