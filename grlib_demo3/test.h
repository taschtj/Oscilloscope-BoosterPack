//*****************************************************************************
// 
// test.h - header file for a compressed string table.
//
// This is an auto-generated file.  Do not edit by hand.
//
//*****************************************************************************

#define SCOMP_MAX_STRLEN         2     // The maximum size of any string.

extern const uint8_t g_pui8Tabletest[];

//*****************************************************************************
//
// SCOMP_STR_INDEX is an enumeration list that is used to select a string
// from the string table using the GrStringGet() function.
//
//*****************************************************************************
enum SCOMP_STR_INDEX
{
    STR_A,                        
    STR_B,                        
    STR_C,                        
};
//*****************************************************************************
//
// The following global variables and #defines are intended to aid in setting
// up codepage mapping tables for use with this string table and an appropriate
// font.  To use only this string table, GrLibInit may be called with a pointer
// to the g_GrLibDefaulttest structure.
//
//*****************************************************************************
extern tCodePointMap g_psCodePointMap_test[];

#define GRLIB_UNICODE_MAP_test {CODEPAGE_ISO8859_1, CODEPAGE_UNICODE, GrMapISO8859_1_Unicode}
#define GRLIB_SELF_MAP_test {CODEPAGE_ISO8859_1, CODEPAGE_ISO8859_1, GrMap8BitIdentity}

extern tGrLibDefaults g_sGrLibDefaulttest;
