//*****************************************************************************
// 
// test.c - Source file for a compressed string table.
//
// This is an auto-generated file.  Do not edit by hand.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "grlib/grlib.h"

//
// The auto-generated string table and associated data.
//
const uint8_t g_pui8Tabletest[] =
{
    //
    // Number of Strings
    //
    0x03, 0x00,

    //
    // Number of Languages
    //
    0x01, 0x00,

    //
    // Language Identifier Table
    //
    ((GrLangEnUS) & 0xff), ((GrLangEnUS) >> 8) & 0xff,

    //
    // Language GrLangEnUS
    //
    0x00, 0x00, 0x00, 0x00, // STR_A
    0x02, 0x00, 0x00, 0x00, // STR_B
    0x04, 0x00, 0x00, 0x00, // STR_C

    //
    // Compressed String Table
    //
     'a', 0x00,  'b', 0x00,  'c', 0x00, 
};

//*****************************************************************************
//
// The following global variables and #defines are intended to aid in setting
// up codepage mapping tables for use with this string table and an appropriate
// font.  To use only this string table, GrLibInit may be called with a pointer
// to the g_GrLibDefaulttest structure.
//
//*****************************************************************************
tCodePointMap g_psCodePointMap_test[] =
{
    {CODEPAGE_ISO8859_1, CODEPAGE_UNICODE, GrMapISO8859_1_Unicode},
    {CODEPAGE_ISO8859_1, CODEPAGE_ISO8859_1, GrMap8BitIdentity}
};

#define NUM_CODEPOINT_MAPS (sizeof(g_psCodePointMap_test) / sizeof(tCodePointMap))

tGrLibDefaults g_sGrLibDefaulttest =
{
    GrDefaultStringRenderer,
    g_psCodePointMap_test,
    CODEPAGE_ISO8859_1,
    NUM_CODEPOINT_MAPS,
    0
};
