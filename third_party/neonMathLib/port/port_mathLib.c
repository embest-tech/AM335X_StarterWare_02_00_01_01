/*
** The code used in this file has been reused from original code available from
** the following URL http://gruntthepeon.free.fr/ssemath/neon_mathfun_test.c
** mentioned in this link http://gruntthepeon.free.fr/ssemath/neon_mathfun.html
**
** Note:
**    1: The functions benchMarkSineRoutine, benchMarkCosineRoutine,
**       CephesMathLibInit required by the benchmark application
**       make use of the code from the above link.
**
*/
#include "neon_mathfun.h"

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
float x = 0.5f, y = 0;
/*
** Vector of 4 float variables to be initialize and measure performance
** of Neon enabled Maths Functions.
**/
v4sf minVal, maxVal, value;

/*
** Wrapper Function to Benchmark the Sine function
** from the Cephes Maths Library.
** This function is called from the benchmarking
** application.
*/
void BenchmarkCephesSine()
{
    int j;

    for (j=0; j < 4; ++j)
    {
        x += 1e-6f;
        y += cephes_sinf(x+5*(j&1));
    }
}

/*
** Wrapper Function to Benchmark the Sine function
** from the Cephes Maths Library.
** This function is called from the benchmarking
** application.
*/
void BenchmarkCephesCosine()
{
    int j;

    for (j=0; j < 4; ++j)
    {
        x += 1e-6f;
        y += cephes_cosf(x+5*(j&1));
    }
}

/*
** Wrapper Function to Initialize the Sine & Cosine function
** implemented using Neon Intrinsics.
** This function is called from the benchmarking
** application.
*/
void CephesMathLibInit()
{
    /*
    ** Initialize the Minimum and Maximum Values
    ** required for the sine and cos maths functions
    ** before benchmarking them.
    */
    minVal = set_ps1(0.5);
    maxVal = set_ps1(1.0);
    value = set_ps1(0.75);
}


/*
** Wrapper Function to Benchmark the Sine function
** implemented using Neon Intrinsics.
** This function is called from the benchmarking
** application.
*/
void BenchmarkIntrinsicSine()
{
    /*
    ** Measure the Performance of Vectorized Maths library
    ** Cosine function.
    */
    value = sin_ps(value);

    value = min_ps(value, maxVal);

    value = max_ps(value, minVal);
}

/*
** Wrapper Function to Benchmark the Sine function
** implemented using Neon Intrinsics.
** This function is called from the benchmarking
** application.
*/
void BenchmarkIntrinsicCosine()
{
    /*
    ** Measure the Performance of Vectorized Maths library
    ** Cosine function.
    */
    value = cos_ps(value);

    value = min_ps(value, maxVal);

    value = max_ps(value, minVal);
}
