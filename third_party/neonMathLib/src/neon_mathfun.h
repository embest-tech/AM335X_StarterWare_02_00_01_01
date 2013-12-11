/* NEON implementation of sin, cos, exp and log

   Inspired by Intel Approximate Math library, and based on the
   corresponding algorithms of the cephes math library
*/

/* Copyright (C) 2011  Julien Pommier

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  (this is the zlib license)

  Note: This file has been modified in the following manner
  1: The Neon optimized maths functions defined here originally have been
     moved to neon_mathfun.c file.
  2: The prototypes of the these functions have been added here.
  3: The prototypes of wrapper functions used for performance benchmarking
     have been added here.
*/

#include <arm_neon.h>

typedef float32x4_t v4sf;  // vector of 4 float
typedef uint32x4_t v4su;  // vector of 4 uint32
typedef int32x4_t v4si;  // vector of 4 uint32

/* Prototypes of Neon Optimized Maths functions */
extern v4sf sin_ps(v4sf x);
extern v4sf cos_ps(v4sf x);
extern void sincos_ps(v4sf x, v4sf *ysin, v4sf *ycos);
extern v4sf exp_ps(v4sf x);
extern v4sf log_ps(v4sf x);
extern v4sf min_ps(v4sf a, v4sf b);
extern v4sf max_ps(v4sf a, v4sf b);
extern v4sf set_ps1(float f);
extern float cephes_sinf(float xx);
extern float cephes_cosf(float xx);

