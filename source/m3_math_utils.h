//
//  m3_math_utils.h
//
//  Created by Volodymyr Shymanksyy on 8/10/19.
//  Copyright © 2019 Volodymyr Shymanskyy. All rights reserved.
//

#ifndef m3_math_utils_h
#define m3_math_utils_h

#include "m3_core.h"

#include <limits.h>

#if defined(M3_COMPILER_MSVC)

#include <intrin.h>

#define __builtin_popcount    __popcnt

static inline
int __builtin_ctz(uint32_t x) {
    unsigned long ret;
    _BitScanForward(&ret, x);
    return (int)ret;
}

static inline
int __builtin_clz(uint32_t x) {
    unsigned long ret;
    _BitScanReverse(&ret, x);
    return (int)(31 ^ ret);
}



#ifdef _WIN64

#define __builtin_popcountll  __popcnt64

static inline
int __builtin_ctzll(uint64_t value) {
    unsigned long ret;
     _BitScanForward64(&ret, value);
    return (int)ret;
}

static inline
int __builtin_clzll(uint64_t value) {
    unsigned long ret;
    _BitScanReverse64(&ret, value);
    return (int)(63 ^ ret);
}

#else // _WIN64

#define __builtin_popcountll(x)  (__popcnt((x) & 0xFFFFFFFF) + __popcnt((x) >> 32))

static inline
int __builtin_ctzll(uint64_t value) {
    //if (value == 0) return 64; // Note: ctz(0) result is undefined anyway
    uint32_t msh = (uint32_t)(value >> 32);
    uint32_t lsh = (uint32_t)(value & 0xFFFFFFFF);
    if (lsh != 0) return __builtin_ctz(lsh);
    return 32 + __builtin_ctz(msh);
}

static inline
int __builtin_clzll(uint64_t value) {
    //if (value == 0) return 64; // Note: clz(0) result is undefined anyway
    uint32_t msh = (uint32_t)(value >> 32);
    uint32_t lsh = (uint32_t)(value & 0xFFFFFFFF);
    if (msh != 0) return __builtin_clz(msh);
    return 32 + __builtin_clz(lsh);
}

#endif // _WIN64

#endif // defined(M3_COMPILER_MSVC)


// TODO: not sure why, signbit is actually defined in math.h
#if (defined(ESP8266) || defined(ESP32)) && !defined(signbit)
    #define signbit(__x) \
            ((sizeof(__x) == sizeof(float))  ?  __signbitf(__x) : __signbitd(__x))
#endif

#if defined(__AVR__)

static inline
float rintf( float arg ) {
  union { float f; uint32_t i; } u;
  u.f = arg;
  uint32_t ux = u.i & 0x7FFFFFFF;
  if (UNLIKELY(ux == 0 || ux > 0x5A000000)) {
    return arg;
  }
  return (float)lrint(arg);
}

static inline
double rint( double arg ) {
  union { double f; uint32_t i[2]; } u;
  u.f = arg;
  uint32_t ux = u.i[1] & 0x7FFFFFFF;
  if (UNLIKELY((ux == 0 && u.i[0] == 0) || ux > 0x433FFFFF)) {
    return arg;
  }
  return (double)lrint(arg);
}

//TODO
static inline
uint64_t strtoull(const char* str, char** endptr, int base) {
  return 0;
}

#endif

/*
 * Rotr, Rotl
 */

static inline
u32 rotl32(u32 n, unsigned c) {
    const unsigned mask = CHAR_BIT * sizeof(n) - 1;
    c &= mask & 31;
    return (n << c) | (n >> ((-c) & mask));
}

static inline
u32 rotr32(u32 n, unsigned c) {
    const unsigned mask = CHAR_BIT * sizeof(n) - 1;
    c &= mask & 31;
    return (n >> c) | (n << ((-c) & mask));
}

static inline
u64 rotl64(u64 n, unsigned c) {
    const unsigned mask = CHAR_BIT * sizeof(n) - 1;
    c &= mask & 63;
    return (n << c) | (n >> ((-c) & mask));
}

static inline
u64 rotr64(u64 n, unsigned c) {
    const unsigned mask = CHAR_BIT * sizeof(n) - 1;
    c &= mask & 63;
    return (n >> c) | (n << ((-c) & mask));
}

/*
 * Integer Div, Rem
 */

#define OP_DIV_U(RES, A, B)                                      \
    if (UNLIKELY(B == 0)) newTrap (m3Err_trapDivisionByZero);    \
    RES = A / B;

#define OP_REM_U(RES, A, B)                                      \
    if (UNLIKELY(B == 0)) newTrap (m3Err_trapDivisionByZero);    \
    RES = A % B;

// 2's complement detection
#if (INT_MIN != -INT_MAX)

    #define OP_DIV_S(RES, A, B, TYPE_MIN)                         \
        if (UNLIKELY(B == 0)) newTrap (m3Err_trapDivisionByZero); \
        if (UNLIKELY(B == -1 and A == TYPE_MIN)) {                \
            newTrap (m3Err_trapIntegerOverflow);                  \
        }                                                         \
        RES = A / B;

    #define OP_REM_S(RES, A, B, TYPE_MIN)                         \
        if (UNLIKELY(B == 0)) newTrap (m3Err_trapDivisionByZero); \
        if (UNLIKELY(B == -1 and A == TYPE_MIN)) RES = 0;         \
        else RES = A % B;

#else

    #define OP_DIV_S(RES, A, B, TYPE_MIN) OP_DIV_U(RES, A, B)
    #define OP_REM_S(RES, A, B, TYPE_MIN) OP_REM_U(RES, A, B)

#endif

// softfloat
//
typedef float32_t   sf32;
typedef float64_t   sf64;

static inline sf32
f32_to_sf(f32 a)
{
    sf32 out;
    out.v = a;
    return out;
}

static inline sf64
f64_to_sf(f64 a)
{
    sf64 out;
    out.v = a;
    return out;
}

static inline f32
f32_of_sf(sf32 a)
{
    return a.v;
}

static inline f64
f64_of_sf(sf64 a)
{
    return a.v;
}

#define TO_SF(TYPE, A)      ( TYPE##_to_sf(A) )
#define OF_SF(TYPE, A)      ( TYPE##_of_sf(A) )

static inline bool
f32_isnan(sf32 a)
{
    return !f32_eq(a, a);
}

static inline bool
f64_isnan(sf64 a)
{
    return !f64_eq(a, a);
}


/*
 * Trunc
 */

#define f32_FULL_EXP  0xFF
#define f64_FULL_EXP  0x7FF

#define f32_LEN_MANT  23
#define f64_LEN_MANT  52

#define f32_EXP(a)   ((u16) ((a)>>f32_LEN_MANT) & f32_FULL_EXP)
#define f64_EXP(a)   ((u16) ((a)>>f64_LEN_MANT) & f64_FULL_EXP)

#define f32_SIGN(a)  ((bool) (a >> 31))
#define f64_SIGN(a)  ((bool) (a >> 63))

#define f32_MANT(a) ((u64) a & 0x007FFFFF)
#define f64_MANT(a) ((u64) a & 0x000FFFFFFFFFFFFFUL)

#define f32_BIAS    127
#define f64_BIAS    1023

#define f32_MAX_MANT  (u64)(((u32)1U << f32_LEN_MANT) - 1)
#define f64_MAX_MANT  (u64)(((u64)1U << f64_LEN_MANT) - 1)

#define _f32_nan        ( (f32)0x7FC00000U )
#define _f64_nan        ( (f64)0x7FF8000000000000UL )

#define _f32_neg_zero   ( (f32)((u32)1 << 31) )
#define _f64_neg_zero   ( (f64)((u64)1 << 63) )

#define _f32_plus_one   ( (f32)f32_BIAS <<  f32_LEN_MANT)
#define _f64_plus_one   ( (f64)f64_BIAS <<  f64_LEN_MANT )

#define _f32_neg_one   ( _f32_neg_zero | _f32_plus_one )
#define _f64_neg_one   ( _f64_neg_zero | _f64_plus_one )

// floating point values that are not representable in (u/i)(32/64) format
// after truncation (rounds to zero)

static const sf32 f32_SMALL_ILLEGAL_u32 = {.v = _f32_neg_one};  // -1
static const sf32 f32_SMALL_ILLEGAL_i32 = {.v=(
    (u32)1 << 31
    | ( (f32)(f32_BIAS + 31) << f32_LEN_MANT )
    | ( (f32)1U )
)};         // -1 * (2^(N-1) + epsilon)

static const sf32 f32_SMALL_ILLEGAL_u64 = {.v = _f32_neg_one};  
static const sf32 f32_SMALL_ILLEGAL_i64 = {.v = (
    (u32)1 << 31
    | ( (f32)(f32_BIAS + 63) << f32_LEN_MANT )
    | (f32) 1U
)};  // -1 * (2^(N-1) + epsilon)

static const sf64 f64_SMALL_ILLEGAL_u32 = {.v = _f64_neg_one};  
static const sf64 f64_SMALL_ILLEGAL_i32 = {.v = (
    (u64)1 << 63
    | ( (f64)(f64_BIAS + 31) << f64_LEN_MANT )
    | ( (f64)1U << (f64_LEN_MANT - 31) )
)};  // -1 * (2^(N-1) + 1)

static const sf64 f64_SMALL_ILLEGAL_u64 = {.v = _f64_neg_one};  
static const sf64 f64_SMALL_ILLEGAL_i64 = {.v = (
    (u64)1 << 63
    | ( (f64)(f64_BIAS + 63) << f64_LEN_MANT )
    | ( (f64)1U )
)};  // -1 * (2^(N-1) + epsilon)

static const sf32 f32_BIG_ILLEGAL_u32 = {.v = ((u32)f32_BIAS + 32) << f32_LEN_MANT};  // 2^N
static const sf32 f32_BIG_ILLEGAL_i32 = {.v = ((u32)f32_BIAS + 31) << f32_LEN_MANT};  // 2^(N-1)

static const sf32 f32_BIG_ILLEGAL_u64 = {.v = ((u32)f32_BIAS + 64) << f32_LEN_MANT};
static const sf32 f32_BIG_ILLEGAL_i64 = {.v = ((u32)f32_BIAS + 63) << f32_LEN_MANT};

static const sf64 f64_BIG_ILLEGAL_u32 = {.v = ((u64)f64_BIAS + 32) << f64_LEN_MANT};
static const sf64 f64_BIG_ILLEGAL_i32 = {.v = ((u64)f64_BIAS + 31) << f64_LEN_MANT};

static const sf64 f64_BIG_ILLEGAL_u64 = {.v = ((u64)f64_BIAS + 64) << f64_LEN_MANT};
static const sf64 f64_BIG_ILLEGAL_i64 = {.v = ((u64)f64_BIAS + 63) << f64_LEN_MANT};



#define TRUNC_FUNC(TYPE_SOURCE, TYPE) _##TYPE##_trunc_##TYPE_SOURCE

#define OP_TRUNC(RES, A, TYPE, TYPE_SOURCE)                         \
    if (UNLIKELY(TYPE_SOURCE##_isnan(TO_SF(TYPE_SOURCE, A)))) {                         \
        newTrap (m3Err_trapIntegerConversion);                      \
    }                                                               \
    if (UNLIKELY(                                                   \
        TYPE_SOURCE##_le(TO_SF(TYPE_SOURCE, A), TYPE_SOURCE##_SMALL_ILLEGAL_##TYPE)     \
        || TYPE_SOURCE##_le(TYPE_SOURCE##_BIG_ILLEGAL_##TYPE, TO_SF(TYPE_SOURCE, A))    \
    )) {                                                            \
        newTrap (m3Err_trapIntegerOverflow);                        \
    }                                                               \
    RES = _##TYPE##_trunc_##TYPE_SOURCE(A);

#define OP_I32_TRUNC_F32(RES, A)    OP_TRUNC(RES, A, i32, f32)
#define OP_U32_TRUNC_F32(RES, A)    OP_TRUNC(RES, A, u32, f32)
#define OP_I32_TRUNC_F64(RES, A)    OP_TRUNC(RES, A, i32, f64)
#define OP_U32_TRUNC_F64(RES, A)    OP_TRUNC(RES, A, u32, f64)

#define OP_I64_TRUNC_F32(RES, A)    OP_TRUNC(RES, A, i64, f32)
#define OP_U64_TRUNC_F32(RES, A)    OP_TRUNC(RES, A, u64, f32)
#define OP_I64_TRUNC_F64(RES, A)    OP_TRUNC(RES, A, i64, f64)
#define OP_U64_TRUNC_F64(RES, A)    OP_TRUNC(RES, A, u64, f64)

#define OP_TRUNC_SAT(RES, A, TYPE, TYPE_SOURCE, IMIN, IMAX)                         \
    if (UNLIKELY(TYPE_SOURCE##_isnan(TO_SF(TYPE_SOURCE, A)))) {                                                       \
        RES = 0;                                                                    \
    } else if (UNLIKELY(TYPE_SOURCE##_le(TO_SF(TYPE_SOURCE, A), TYPE_SOURCE##_SMALL_ILLEGAL_##TYPE))) { \
        RES = IMIN;                                                                 \
    } else if (UNLIKELY(TYPE_SOURCE##_le(TYPE_SOURCE##_BIG_ILLEGAL_##TYPE, TO_SF(TYPE_SOURCE, A)))) {   \
        RES = IMAX;                                                                 \
    } else {                                                                        \
        RES = _##TYPE##_trunc_##TYPE_SOURCE(A);                                     \
    }


#define OP_I32_TRUNC_SAT_F32(RES, A)    OP_TRUNC_SAT(RES, A, i32, f32, INT32_MIN, INT32_MAX)
#define OP_U32_TRUNC_SAT_F32(RES, A)    OP_TRUNC_SAT(RES, A, u32, f32, (u32)0,    UINT32_MAX)
#define OP_I32_TRUNC_SAT_F64(RES, A)    OP_TRUNC_SAT(RES, A, i32, f64, INT32_MIN, INT32_MAX)
#define OP_U32_TRUNC_SAT_F64(RES, A)    OP_TRUNC_SAT(RES, A, u32, f64, (u32)0,    UINT32_MAX)

#define OP_I64_TRUNC_SAT_F32(RES, A)    OP_TRUNC_SAT(RES, A, i64, f32, INT64_MIN, INT64_MAX)
#define OP_U64_TRUNC_SAT_F32(RES, A)    OP_TRUNC_SAT(RES, A, u64, f32, (u64)0,    UINT64_MAX)
#define OP_I64_TRUNC_SAT_F64(RES, A)    OP_TRUNC_SAT(RES, A, i64, f64, INT64_MIN, INT64_MAX)
#define OP_U64_TRUNC_SAT_F64(RES, A)    OP_TRUNC_SAT(RES, A, u64, f64, (u64)0,    UINT64_MAX)

/*
 * Min, Max
 */

#if d_m3HasFloat

#define _nan_unify(TYPE)            \
    static inline TYPE              \
    _##TYPE##_nan_unify(TYPE a)     \
    {                               \
        if (TYPE##_isnan(TO_SF(TYPE, a)))        \
        {                           \
            return _##TYPE##_nan;   \
        }                           \
        return a;                   \
    }
_nan_unify(f32)
_nan_unify(f64)

#define _ne(TYPE)                       \
    static inline bool                  \
    _##TYPE##_ne(TYPE a, TYPE b)        \
    {                                   \
        return !(TYPE##_eq(TO_SF(TYPE, a), TO_SF(TYPE, b)));      \
    }
_ne(f32)
_ne(f64)

#define _eq(TYPE)                       \
    static inline bool                  \
    _##TYPE##_eq(TYPE a, TYPE b)        \
    {                                   \
        return (TYPE##_eq(TO_SF(TYPE, a), TO_SF(TYPE, b)));      \
    }
_eq(f32)
_eq(f64)

#define _lt(TYPE)                       \
    static inline bool                  \
    _##TYPE##_lt(TYPE a, TYPE b)        \
    {                                   \
        return (TYPE##_lt(TO_SF(TYPE, a), TO_SF(TYPE, b)));       \
    }
_lt(f32)
_lt(f64)

// f32_le
#define _le(TYPE)                       \
    static inline bool                  \
    _##TYPE##_le(TYPE a, TYPE b)        \
    {                                   \
        return (TYPE##_le(TO_SF(TYPE, a), TO_SF(TYPE, b)));       \
    }
_le(f32)
_le(f64)

#define _gt(TYPE)                       \
    static inline bool                  \
    _##TYPE##_gt(TYPE a, TYPE b)        \
    {                                   \
        return (TYPE##_lt(TO_SF(TYPE, b), TO_SF(TYPE, a)));       \
    }
_gt(f32)
_gt(f64)

#define _ge(TYPE)                       \
    static inline bool                  \
    _##TYPE##_ge(TYPE a, TYPE b)        \
    {                                   \
        return (TYPE##_le(TO_SF(TYPE, b), TO_SF(TYPE, a)));       \
    }
_ge(f32)
_ge(f64)

#define _add(TYPE)                                          \
    static inline TYPE                                      \
    _##TYPE##_add(TYPE a, TYPE b)                           \
    {                                                       \
        softfloat_roundingMode = softfloat_round_near_even; \
        return _##TYPE##_nan_unify(OF_SF(TYPE, TYPE##_add(TO_SF(TYPE, a), TO_SF(TYPE, b))));       \
    }
_add(f32)
_add(f64)
    
#define _mul(TYPE)                                          \
    static inline TYPE                                      \
    _##TYPE##_mul(TYPE a, TYPE b)                           \
    {                                                       \
        softfloat_roundingMode = softfloat_round_near_even; \
        return _##TYPE##_nan_unify(OF_SF(TYPE, TYPE##_mul(TO_SF(TYPE, a), TO_SF(TYPE, b))));       \
    }
_mul(f32)
_mul(f64)

#define _sub(TYPE)                                          \
    static inline TYPE                                      \
    _##TYPE##_sub(TYPE a, TYPE b)                           \
    {                                                       \
        softfloat_roundingMode = softfloat_round_near_even; \
        return _##TYPE##_nan_unify(OF_SF(TYPE, TYPE##_sub(TO_SF(TYPE, a), TO_SF(TYPE, b))));       \
    }
_sub(f32)
_sub(f64)

#define _div(TYPE)                                          \
    static inline TYPE                                      \
    _##TYPE##_div(TYPE a, TYPE b)                           \
    {                                                       \
        softfloat_roundingMode = softfloat_round_near_even; \
        return _##TYPE##_nan_unify(OF_SF(TYPE, TYPE##_div(TO_SF(TYPE, a), TO_SF(TYPE, b))));       \
    }
_div(f32)
_div(f64)

#define _min(TYPE)                                      \
    static inline TYPE                                  \
    _##TYPE##_min(TYPE a, TYPE b)                       \
    {                                                   \
        if (TYPE##_isnan(TO_SF(TYPE, a)))                            \
        {                                               \
            return a;                                   \
        }                                               \
        if (TYPE##_isnan(TO_SF(TYPE, b)))                            \
        {                                               \
            return b;                                   \
        }                                               \
        return (TYPE##_lt(TO_SF(TYPE, a), TO_SF(TYPE, b))) ? a : b;               \
    }
_min(f32)
_min(f64)

#define _max(TYPE)                                      \
    static inline TYPE                                  \
    _##TYPE##_max(TYPE a, TYPE b)                       \
    {                                                   \
        if (TYPE##_isnan(TO_SF(TYPE, a)))                            \
        {                                               \
            return a;                                   \
        }                                               \
        if (TYPE##_isnan(TO_SF(TYPE, b)))                            \
        {                                               \
            return b;                                   \
        }                                               \
        return (TYPE##_lt(TO_SF(TYPE, a), TO_SF(TYPE, b))) ? b : a;               \
    }
_max(f32)
_max(f64)

#define _abs(TYPE)                                      \
    static inline TYPE                                  \
    _##TYPE##_abs(TYPE a)                               \
    {                                                   \
        return a & ~_##TYPE##_neg_zero;                 \
    }
_abs(f32)
_abs(f64)

#define _ceil(TYPE)                                                             \
    static TYPE                                                                 \
    _##TYPE##_ceil(TYPE a)                                                      \
    {                                                                           \
        u16 exp = TYPE##_EXP(a);                                                \
        if ( exp ==  TYPE##_FULL_EXP)                                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u64 mant = TYPE##_MANT(a);                                              \
        if ((exp == 0) && (mant == 0))                                          \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        if (exp < TYPE##_BIAS)                                                  \
        {                                                                       \
            return ( TYPE##_SIGN(a) ) ? _##TYPE##_neg_zero                      \
                                      : _##TYPE##_plus_one;                     \
        }                                                                       \
        if ( (exp - TYPE##_BIAS) >= TYPE##_LEN_MANT )                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u8 fraction_part = (TYPE##_LEN_MANT - (exp - TYPE##_BIAS));             \
        u64 fraction_mask = (1 << fraction_part) - 1;                           \
        if (!(fraction_mask & a))                                               \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        if (TYPE##_SIGN(a))                                                     \
        {                                                                       \
            return ((a >> fraction_part) << fraction_part);                     \
        }                                                                       \
        mant = ((mant >> fraction_part) + 1) << fraction_part;                  \
        if (mant > TYPE##_MAX_MANT)                                             \
        {                                                                       \
            return ((TYPE)(exp + 1) << TYPE##_LEN_MANT);                       \
        }                                                                       \
        return (TYPE) (((a >> TYPE##_LEN_MANT) << TYPE##_LEN_MANT) | mant);     \
    }
_ceil(f32)
_ceil(f64)

#define _floor(TYPE)                                                            \
    static TYPE                                                                 \
    _##TYPE##_floor(TYPE a)                                                     \
    {                                                                           \
        u16 exp = TYPE##_EXP(a);                                                \
        if ( exp ==  TYPE##_FULL_EXP)                                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u64 mant = TYPE##_MANT(a);                                              \
        if ((exp == 0) && (mant == 0))                                          \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        if (exp < TYPE##_BIAS)                                                  \
        {                                                                       \
            return ( TYPE##_SIGN(a) ) ? _##TYPE##_neg_one                       \
                                      : 0;                                      \
        }                                                                       \
        if ( (exp - TYPE##_BIAS) >= TYPE##_LEN_MANT )                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u8 fraction_part = (TYPE##_LEN_MANT - (exp - TYPE##_BIAS));             \
        u64 fraction_mask = (1 << fraction_part) - 1;                           \
        if (!(fraction_mask & a))                                               \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        if (!TYPE##_SIGN(a))                                                    \
        {                                                                       \
            return ((a >> fraction_part) << fraction_part);                     \
        }                                                                       \
        mant = ((mant >> fraction_part) + 1) << fraction_part;                  \
        if (mant > TYPE##_MAX_MANT)                                             \
        {                                                                       \
            return (_##TYPE##_neg_zero | ((TYPE)(exp + 1) << TYPE##_LEN_MANT)); \
        }                                                                       \
        return (TYPE) (((a >> TYPE##_LEN_MANT) << TYPE##_LEN_MANT) | mant);     \
    }
_floor(f32)
_floor(f64)

#define _trunc(TYPE)                                                            \
    static TYPE                                                                 \
    _##TYPE##_trunc(TYPE a)                                                     \
    {                                                                           \
        u16 exp = TYPE##_EXP(a);                                                \
        if ( exp ==  TYPE##_FULL_EXP)                                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u64 mant = TYPE##_MANT(a);                                              \
        if ((exp == 0) && (mant == 0))                                          \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        if (exp < TYPE##_BIAS)                                                  \
        {                                                                       \
            return ( TYPE##_SIGN(a) ) ? _##TYPE##_neg_zero                      \
                                      : 0;                                      \
        }                                                                       \
        if ( (exp - TYPE##_BIAS) >= TYPE##_LEN_MANT )                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u8 fraction_part = (TYPE##_LEN_MANT - (exp - TYPE##_BIAS));             \
        return ((a >> fraction_part) << fraction_part);                         \
    }
_trunc(f32)
_trunc(f64)

#define _sqrt(TYPE)                                         \
    static inline TYPE                                      \
    _##TYPE##_sqrt(TYPE a)                                  \
    {                                                       \
        softfloat_roundingMode = softfloat_round_near_even; \
        return _##TYPE##_nan_unify(OF_SF(TYPE, TYPE##_sqrt(TO_SF(TYPE, a))));         \
    }
_sqrt(f32)
_sqrt(f64)

#define _near(TYPE)                                                             \
    static TYPE                                                                 \
    _##TYPE##_near(TYPE a)                                                      \
    {                                                                           \
        u16 exp = TYPE##_EXP(a);                                                \
        if ( exp ==  TYPE##_FULL_EXP)                                           \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        u64 mant = TYPE##_MANT(a);                                              \
        if ((exp == 0) && (mant == 0))                                          \
        {                                                                       \
            return a;                                                           \
        }                                                                       \
        TYPE floor = _##TYPE##_floor(a);                                        \
        TYPE ceil = _##TYPE##_ceil(a);                                          \
        TYPE floor_diff = OF_SF(TYPE, TYPE##_sub(TO_SF(TYPE, a), TO_SF(TYPE, floor)));                                 \
        TYPE ceil_diff = OF_SF(TYPE, TYPE##_sub(TO_SF(TYPE, ceil), TO_SF(TYPE, a)));                                   \
        if (TYPE##_eq(TO_SF(TYPE, floor_diff), TO_SF(TYPE, ceil_diff)))                                   \
        {                                                                       \
            return (TYPE##_eq(TO_SF(TYPE, 0), TYPE##_rem(TO_SF(TYPE, ceil), TO_SF(TYPE, 2)))) ? ceil : floor;          \
        }                                                                       \
        return (TYPE##_lt(TO_SF(TYPE, ceil_diff), TO_SF(TYPE, floor_diff))) ? ceil : floor;               \
    }
_near(f32)
_near(f64)

// _f32_neg
#define _neg(TYPE)                      \
    static inline TYPE                  \
    _##TYPE##_neg(TYPE a)               \
    {                                   \
        return a ^ _##TYPE##_neg_zero;   \
    }
_neg(f32)
_neg(f64)

// _f32_copysign
#define _copysign(TYPE)                                                 \
    static inline TYPE                                                  \
    _##TYPE##_copysign(TYPE a, TYPE b)                                  \
    {                                                                   \
        return (a & ~_##TYPE##_neg_zero) | (b & _##TYPE##_neg_zero);    \
    }
_copysign(f32)
_copysign(f64)

#define _f32_to_u32      f32_to_ui32_r_minMag
#define _f32_to_u64      f32_to_ui64_r_minMag
#define _f64_to_u32      f64_to_ui32_r_minMag
#define _f64_to_u64      f64_to_ui64_r_minMag

#define _f32_to_i32      f32_to_i32_r_minMag
#define _f32_to_i64      f32_to_i64_r_minMag
#define _f64_to_i32      f64_to_i32_r_minMag
#define _f64_to_i64      f64_to_i64_r_minMag

#define _trunc_convert(TYPE, TYPE_SOURCE)                                       \
    static TYPE                                                                 \
    _##TYPE##_trunc_##TYPE_SOURCE(TYPE_SOURCE a)                                \
    {                                                                           \
        return _##TYPE_SOURCE##_to_##TYPE(TO_SF(TYPE_SOURCE, a), 0);                                \
    }
_trunc_convert(u32, f32)
_trunc_convert(i32, f32)
_trunc_convert(u64, f32)
_trunc_convert(i64, f32)

_trunc_convert(u32, f64)
_trunc_convert(i32, f64)
_trunc_convert(u64, f64)
_trunc_convert(i64, f64)

#define _modify(TO, FROM)                                                       \
    static inline TO                                                            \
    _##TO##_modify_##FROM(FROM a)                                               \
    {                                                                           \
        return OF_SF(TO, FROM##_to_##TO(TO_SF(FROM, a)));                                               \
    }
_modify(f32, f64)
_modify(f64, f32)

// _FLOAT_convert_INT

#define _i32_to_f32     i32_to_f32
#define _i64_to_f32     i64_to_f32
#define _i32_to_f64     i32_to_f64
#define _i64_to_f64     i64_to_f64

#define _u32_to_f32     ui32_to_f32
#define _u64_to_f32     ui64_to_f32
#define _u32_to_f64     ui32_to_f64
#define _u64_to_f64     ui64_to_f64

#define _convert(TO, FROM)                                                      \
    static inline TO                                                            \
    _##TO##_convert_##FROM(FROM a)                                              \
    {                                                                           \
        return OF_SF(TO, _##FROM##_to_##TO(a));                                            \
    }
_convert(f32, u32)
_convert(f32, i32)
_convert(f32, u64)
_convert(f32, i64)

_convert(f64, u32)
_convert(f64, i32)
_convert(f64, u64)
_convert(f64, i64)

#endif  // d_m3HasFloat

#endif // m3_math_utils_h
