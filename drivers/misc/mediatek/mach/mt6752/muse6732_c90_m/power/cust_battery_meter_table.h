#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 0
#define BAT_NTC_47 0
#define BAT_NTC_68 1

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             24000
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#endif

#if (BAT_NTC_68 == 1)
#define RBAT_PULL_UP_R             62000	/* 62K */
#endif

#define RBAT_PULL_UP_VOLT          2800



// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}
    };
#endif

#if (BAT_NTC_68 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,738931},
        {-15,547472},
        {-10,409600},
        { -5,309299},
        {  0,235622},
        {  5,181001},
        { 10,140153},
        { 15,109349},
        { 20,85934},
        { 25,68000},
        { 30,54165},
        { 35,43418},
        { 40,35014},
        { 45,28400},
        { 50,23164},
        { 55,17733},
        { 60,15656},
        { 65,12967},
        { 70,10791},
        { 75,9021},
        { 80,7574}
    };
#endif

#ifdef CONFIG_LGE_PM_BATTERY_PROFILE
/* used in cust_battery_meter_table.h */
#define INIT_BATTERY_PROFILE(maker) \
static int Q_MAX_POS_50 = Q_MAX_POS_50_##maker; \
static int Q_MAX_POS_25 = Q_MAX_POS_25_##maker; \
static int Q_MAX_POS_0 = Q_MAX_POS_0_##maker; \
static int Q_MAX_NEG_10 = Q_MAX_NEG_10_##maker; \
static int Q_MAX_POS_50_H_CURRENT = Q_MAX_POS_50_H_CURRENT_##maker; \
static int Q_MAX_POS_25_H_CURRENT = Q_MAX_POS_25_H_CURRENT_##maker; \
static int Q_MAX_POS_0_H_CURRENT = Q_MAX_POS_0_H_CURRENT_##maker; \
static int Q_MAX_NEG_10_H_CURRENT = Q_MAX_NEG_10_H_CURRENT_##maker; \
static int battery_profile_size = BATTERY_PROFILE_SIZE_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t0 = battery_profile_t0_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t1 = battery_profile_t1_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t2 = battery_profile_t2_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t3 = battery_profile_t3_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_temperature = battery_profile_temperature_##maker; \
static int r_profile_size = R_PROFILE_SIZE_##maker; \
static R_PROFILE_STRUC *r_profile_t0 = r_profile_t0_##maker; \
static R_PROFILE_STRUC *r_profile_t1 = r_profile_t1_##maker; \
static R_PROFILE_STRUC *r_profile_t2 = r_profile_t2_##maker; \
static R_PROFILE_STRUC *r_profile_t3 = r_profile_t3_##maker; \
static R_PROFILE_STRUC *r_profile_temperature = r_profile_temperature_##maker;

#define DECLARE_PROFILE(id, maker) \
static int Q_MAX_POS_50_##id = Q_MAX_POS_50_##maker; \
static int Q_MAX_POS_25_##id = Q_MAX_POS_25_##maker; \
static int Q_MAX_POS_0_##id = Q_MAX_POS_0_##maker; \
static int Q_MAX_NEG_10_##id = Q_MAX_NEG_10_##maker; \
static int Q_MAX_POS_50_H_CURRENT_##id = Q_MAX_POS_50_H_CURRENT_##maker; \
static int Q_MAX_POS_25_H_CURRENT_##id = Q_MAX_POS_25_H_CURRENT_##maker; \
static int Q_MAX_POS_0_H_CURRENT_##id = Q_MAX_POS_0_H_CURRENT_##maker; \
static int Q_MAX_NEG_10_H_CURRENT_##id = Q_MAX_NEG_10_H_CURRENT_##maker; \
static int battery_profile_size_##id = BATTERY_PROFILE_SIZE_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t0_##id = battery_profile_t0_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t1_##id = battery_profile_t1_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t2_##id = battery_profile_t2_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t3_##id = battery_profile_t3_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_temperature_##id = battery_profile_temperature_##maker; \
static int r_profile_size_##id = R_PROFILE_SIZE_##maker; \
static R_PROFILE_STRUC *r_profile_t0_##id = r_profile_t0_##maker; \
static R_PROFILE_STRUC *r_profile_t1_##id = r_profile_t1_##maker; \
static R_PROFILE_STRUC *r_profile_t2_##id = r_profile_t2_##maker; \
static R_PROFILE_STRUC *r_profile_t3_##id = r_profile_t3_##maker; \
static R_PROFILE_STRUC *r_profile_temperature_##id = r_profile_temperature_##maker;

/* used in battery_meter.c */
#define SET_BATTERY_PROFILE(id) \
{ \
	Q_MAX_POS_50 = Q_MAX_POS_50_##id; \
	Q_MAX_POS_25 = Q_MAX_POS_25_##id; \
	Q_MAX_POS_0 = Q_MAX_POS_0_##id; \
	Q_MAX_NEG_10 = Q_MAX_NEG_10_##id; \
	Q_MAX_POS_50_H_CURRENT = Q_MAX_POS_50_H_CURRENT_##id; \
	Q_MAX_POS_25_H_CURRENT = Q_MAX_POS_25_H_CURRENT_##id; \
	Q_MAX_POS_0_H_CURRENT = Q_MAX_POS_0_H_CURRENT_##id; \
	Q_MAX_NEG_10_H_CURRENT = Q_MAX_NEG_10_H_CURRENT_##id; \
	battery_profile_size = battery_profile_size_##id; \
	battery_profile_t0 = battery_profile_t0_##id; \
	battery_profile_t1 = battery_profile_t1_##id; \
	battery_profile_t2 = battery_profile_t2_##id; \
	battery_profile_t3 = battery_profile_t3_##id; \
	battery_profile_temperature = battery_profile_temperature_##id; \
	r_profile_size = r_profile_size_##id; \
	r_profile_t0 = r_profile_t0_##id; \
	r_profile_t1 = r_profile_t1_##id; \
	r_profile_t2 = r_profile_t2_##id; \
	r_profile_t3 = r_profile_t3_##id; \
	r_profile_temperature = r_profile_temperature_##id; \
}

#include <cust_battery_meter_table_LGC.h>
#include <cust_battery_meter_table_TOCAD.h>

/* Default Battery Profile */
INIT_BATTERY_PROFILE(LGC);

/* ID - Maker Maching */
DECLARE_PROFILE(DS2704_N, TOCAD);		/* Not used */
DECLARE_PROFILE(DS2704_L, LGC);
DECLARE_PROFILE(DS2704_C, TOCAD);
DECLARE_PROFILE(ISL6296_N, LGC);		/* Not used */
DECLARE_PROFILE(ISL6296_L, TOCAD);
DECLARE_PROFILE(ISL6296_C, LGC);
DECLARE_PROFILE(RA4301_VC0, LGC);		/* Not used */
DECLARE_PROFILE(RA4301_VC1, LGC);		/* Not used */
DECLARE_PROFILE(RA4301_VC2, LGC);		/* Not used */
DECLARE_PROFILE(SW3800_VC0, LGC);		/* Not used */
DECLARE_PROFILE(SW3800_VC1, LGC);		/* Not used */
DECLARE_PROFILE(SW3800_VC2, LGC);		/* Not used */
#else
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
    {0  ,	4309},
    {2  ,	4248},
    {5  ,	4214},
    {7  ,	4188},
    {9  ,	4165},
    {12 ,	4141},
    {14 ,	4120},
    {16 ,	4100},
    {18 ,	4081},
    {21 ,	4059},
    {23 ,	4035},
    {25 ,	4012},
    {28 ,	3990},
    {30 ,	3968},
    {32 ,	3948},
    {35 ,	3930},
    {37 ,	3912},
    {39 ,	3894},
    {41 ,	3879},
    {44 ,	3866},
    {46 ,	3854},
    {48 ,	3842},
    {51 ,	3832},
    {53 ,	3824},
    {55 ,	3816},
    {58 ,	3807},
    {60 ,	3800},
    {62 ,	3793},
    {64 ,	3790},
    {67 ,	3784},
    {69 ,	3776},
    {71 ,	3769},
    {74 ,	3755},
    {76 ,	3739},
    {78 ,	3722},
    {81 ,	3711},
    {83 ,	3704},
    {85 ,	3696},
    {87 ,	3680},
    {90 ,	3626},
    {92 ,	3551},
    {94 ,	3465},
    {95 ,	3430},
    {96 ,	3410},
    {97 ,	3393},
    {97 ,	3381},
    {98 ,	3372},
    {98 ,	3362},
    {98 ,	3352},
    {98 ,	3347},
    {99 ,	3342},
    {99 ,	3335},
    {99 ,	3330},
    {99 ,	3324},
    {99 ,	3320},
    {100    ,	3316},
    {100    ,	3313},
    {100    ,	3308},
    {100    ,	3305},
    {100    ,	3300},
    {100    ,	3297},
    {100    ,	3293},
    {100    ,	3290},
    {100    ,	3287},
    {100    ,	3284},
    {100    ,	3282},
    {100    ,	3277},
    {100    ,	3275},
    {100    ,	3273}
};

// T1 0C
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
    {0  ,	4324},
    {2  ,	4289},
    {4  ,	4262},
    {6  ,	4237},
    {9  ,	4214},
    {11 ,	4191},
    {13 ,	4169},
    {15 ,	4147},
    {17 ,	4126},
    {19 ,	4104},
    {22 ,	4086},
    {24 ,	4067},
    {26 ,	4045},
    {28 ,	4023},
    {30 ,	4003},
    {32 ,	3981},
    {35 ,	3959},
    {37 ,	3936},
    {39 ,	3918},
    {41 ,	3900},
    {43 ,	3885},
    {45 ,	3871},
    {48 ,	3860},
    {50 ,	3850},
    {52 ,	3839},
    {54 ,	3831},
    {56 ,	3823},
    {58 ,	3813},
    {61 ,	3806},
    {63 ,	3799},
    {65 ,	3791},
    {67 ,	3784},
    {69 ,	3775},
    {71 ,	3760},
    {74 ,	3743},
    {76 ,	3724},
    {78 ,	3710},
    {80 ,	3704},
    {82 ,	3699},
    {84 ,	3692},
    {87 ,	3653},
    {89 ,	3601},
    {91 ,	3553},
    {93 ,	3502},
    {95 ,	3445},
    {97 ,	3388},
    {98 ,	3354},
    {99 ,	3326},
    {100    ,	3304},
    {100    ,	3283},
    {100    ,	3265},
    {100    ,	3249},
    {100    ,	3234},
    {100    ,	3222},
    {100    ,	3211},
    {100    ,	3200},
    {100    ,	3189},
    {100    ,	3180},
    {100    ,	3171},
    {100    ,	3161},
    {100    ,	3154},
    {100    ,	3146},
    {100    ,	3139},
    {100    ,	3131},
    {100    ,	3124},
    {100    ,	3120},
    {100    ,	3114},
    {100    ,	3110},
    {100    ,	3106}
};

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
    {0  ,	4338},
    {2  ,	4309},
    {4  ,	4286},
    {6  ,	4261},
    {8  ,	4238},
    {11 ,	4216},
    {13 ,	4192},
    {15 ,	4170},
    {17 ,	4148},
    {19 ,	4129},
    {21 ,	4107},
    {23 ,	4087},
    {25 ,	4069},
    {27 ,	4050},
    {29 ,	4029},
    {31 ,	4011},
    {34 ,	3993},
    {36 ,	3976},
    {38 ,	3956},
    {40 ,	3931},
    {42 ,	3909},
    {44 ,	3891},
    {46 ,	3877},
    {48 ,	3865},
    {50 ,	3855},
    {53 ,	3844},
    {55 ,	3834},
    {57 ,	3827},
    {59 ,	3817},
    {61 ,	3808},
    {63 ,	3801},
    {65 ,	3793},
    {67 ,	3781},
    {69 ,	3768},
    {71 ,	3754},
    {73 ,	3738},
    {76 ,	3717},
    {78 ,	3699},
    {80 ,	3694},
    {82 ,	3691},
    {84 ,	3686},
    {86 ,	3666},
    {88 ,	3639},
    {90 ,	3600},
    {92 ,	3550},
    {94 ,	3494},
    {97 ,	3439},
    {99 ,	3372},
    {100    ,	3260},
    {100    ,	3154},
    {100    ,	3086},
    {100    ,	3038},
    {100    ,	3003},
    {100    ,	2978},
    {100    ,	2962},
    {100    ,	2948},
    {100    ,	2938},
    {100    ,	2932},
    {100    ,	2924},
    {100    ,	2917},
    {100    ,	2913},
    {100    ,	2909},
    {100    ,	2907},
    {100    ,	2901},
    {100    ,	2897},
    {100    ,	2896},
    {100    ,	2896},
    {100    ,	2891},
    {100    ,	2893}
};

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
    {0  ,	4339},
    {2  ,	4310},
    {4  ,	4283},
    {6  ,	4259},
    {9  ,	4235},
    {11 ,	4211},
    {13 ,	4186},
    {15 ,	4163},
    {17 ,	4141},
    {19 ,	4120},
    {21 ,	4098},
    {24 ,	4078},
    {26 ,	4056},
    {28 ,	4038},
    {30 ,	4019},
    {32 ,	4001},
    {34 ,	3982},
    {37 ,	3964},
    {39 ,	3938},
    {41 ,	3914},
    {43 ,	3895},
    {45 ,	3881},
    {47 ,	3867},
    {49 ,	3857},
    {52 ,	3845},
    {54 ,	3835},
    {56 ,	3825},
    {58 ,	3816},
    {60 ,	3806},
    {62 ,	3788},
    {64 ,	3775},
    {67 ,	3762},
    {69 ,	3748},
    {71 ,	3735},
    {73 ,	3715},
    {75 ,	3695},
    {77 ,	3683},
    {80 ,	3678},
    {82 ,	3675},
    {84 ,	3665},
    {86 ,	3645},
    {88 ,	3617},
    {90 ,	3577},
    {92 ,	3530},
    {95 ,	3479},
    {97 ,	3429},
    {99 ,	3360},
    {100    ,	3252},
    {100    ,	3135},
    {100    ,	3078},
    {100    ,	3034},
    {100    ,	3004},
    {100    ,	3005},
    {100    ,	2971},
    {100    ,	2958},
    {100    ,	2944},
    {100    ,	2931},
    {100    ,	2924},
    {100    ,	2917},
    {100    ,	2913},
    {100    ,	2907},
    {100    ,	2903},
    {100    ,	2900},
    {100    ,	2896},
    {100    ,	2901},
    {100    ,	2890},
    {100    ,	2889},
    {100    ,	2887},
    {100    ,	2886}
};

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
    {570    ,	4309},
    {570    ,	4248},
    {610    ,	4214},
    {628    ,	4188},
    {650    ,	4165},
    {670    ,	4141},
    {673    ,	4120},
    {688    ,	4100},
    {728    ,	4081},
    {745    ,	4059},
    {750    ,	4035},
    {758    ,	4012},
    {753    ,	3990},
    {735    ,	3968},
    {733    ,	3948},
    {745    ,	3930},
    {738    ,	3912},
    {738    ,	3894},
    {755    ,	3879},
    {753    ,	3866},
    {773    ,	3854},
    {773    ,	3842},
    {788    ,	3832},
    {808    ,	3824},
    {813    ,	3816},
    {835    ,	3807},
    {835    ,	3800},
    {865    ,	3793},
    {883    ,	3790},
    {903    ,	3784},
    {935    ,	3776},
    {950    ,	3769},
    {980    ,	3755},
    {1000   ,	3739},
    {1018   ,	3722},
    {1070   ,	3711},
    {1118   ,	3704},
    {1178   ,	3696},
    {1280   ,	3680},
    {1330   ,	3626},
    {1438   ,	3551},
    {1645   ,	3465},
    {1578   ,	3430},
    {1528   ,	3410},
    {1485   ,	3393},
    {1458   ,	3381},
    {1430   ,	3372},
    {1413   ,	3362},
    {1380   ,	3352},
    {1370   ,	3347},
    {1358   ,	3342},
    {1348   ,	3335},
    {1328   ,	3330},
    {1315   ,	3324},
    {1310   ,	3320},
    {1290   ,	3316},
    {1290   ,	3313},
    {1278   ,	3308},
    {1270   ,	3305},
    {1263   ,	3300},
    {1258   ,	3297},
    {1240   ,	3293},
    {1233   ,	3290},
    {1223   ,	3287},
    {1213   ,	3284},
    {1215   ,	3282},
    {1205   ,	3277},
    {1203   ,	3275},
    {1203   ,	3273}
};

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
    {345    ,	4324},
    {345    ,	4289},
    {355    ,	4262},
    {365    ,	4237},
    {365    ,	4214},
    {368    ,	4191},
    {380    ,	4169},
    {383    ,	4147},
    {395    ,	4126},
    {390    ,	4104},
    {403    ,	4086},
    {430    ,	4067},
    {443    ,	4045},
    {438    ,	4023},
    {445    ,	4003},
    {430    ,	3981},
    {413    ,	3959},
    {398    ,	3936},
    {398    ,	3918},
    {393    ,	3900},
    {388    ,	3885},
    {388    ,	3871},
    {400    ,	3860},
    {405    ,	3850},
    {413    ,	3839},
    {415    ,	3831},
    {425    ,	3823},
    {433    ,	3813},
    {440    ,	3806},
    {448    ,	3799},
    {453    ,	3791},
    {465    ,	3784},
    {473    ,	3775},
    {480    ,	3760},
    {483    ,	3743},
    {495    ,	3724},
    {518    ,	3710},
    {535    ,	3704},
    {583    ,	3699},
    {645    ,	3692},
    {703    ,	3653},
    {830    ,	3601},
    {948    ,	3553},
    {1075   ,	3502},
    {1228   ,	3445},
    {1470   ,	3388},
    {1388   ,	3354},
    {1320   ,	3326},
    {1273   ,	3304},
    {1210   ,	3283},
    {1175   ,	3265},
    {1125   ,	3249},
    {1093   ,	3234},
    {1058   ,	3222},
    {1035   ,	3211},
    {1015   ,	3200},
    {983    ,	3189},
    {958    ,	3180},
    {930    ,	3171},
    {923    ,	3161},
    {895    ,	3154},
    {873    ,	3146},
    {863    ,	3139},
    {853    ,	3131},
    {823    ,	3124},
    {813    ,	3120},
    {800    ,	3114},
    {790    ,	3110},
    {765    ,	3106}
};

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
    {135    ,	4338},
    {135    ,	4309},
    {140    ,	4286},
    {140    ,	4261},
    {143    ,	4238},
    {150    ,	4216},
    {148    ,	4192},
    {150    ,	4170},
    {153    ,	4148},
    {163    ,	4129},
    {163    ,	4107},
    {163    ,	4087},
    {170    ,	4069},
    {183    ,	4050},
    {183    ,	4029},
    {190    ,	4011},
    {198    ,	3993},
    {198    ,	3976},
    {190    ,	3956},
    {170    ,	3931},
    {153    ,	3909},
    {145    ,	3891},
    {143    ,	3877},
    {143    ,	3865},
    {150    ,	3855},
    {148    ,	3844},
    {153    ,	3834},
    {160    ,	3827},
    {160    ,	3817},
    {160    ,	3808},
    {163    ,	3801},
    {170    ,	3793},
    {163    ,	3781},
    {158    ,	3768},
    {155    ,	3754},
    {155    ,	3738},
    {155    ,	3717},
    {150    ,	3699},
    {158    ,	3694},
    {168    ,	3691},
    {183    ,	3686},
    {225    ,	3666},
    {273    ,	3639},
    {293    ,	3600},
    {315    ,	3550},
    {363    ,	3494},
    {428    ,	3439},
    {498    ,	3372},
    {673    ,	3260},
    {888    ,	3154},
    {725    ,	3086},
    {605    ,	3038},
    {510    ,	3003},
    {463    ,	2978},
    {420    ,	2962},
    {383    ,	2948},
    {365    ,	2938},
    {330    ,	2932},
    {328    ,	2924},
    {313    ,	2917},
    {298    ,	2913},
    {283    ,	2909},
    {285    ,	2907},
    {283    ,	2901},
    {273    ,	2897},
    {258    ,	2896},
    {243    ,	2896},
    {263    ,	2891},
    {240    ,	2893}
};

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
    {103    ,	4339},
    {103    ,	4310},
    {105    ,	4283},
    {110    ,	4259},
    {115    ,	4235},
    {115    ,	4211},
    {118    ,	4186},
    {115    ,	4163},
    {120    ,	4141},
    {125    ,	4120},
    {125    ,	4098},
    {130    ,	4078},
    {128    ,	4056},
    {138    ,	4038},
    {140    ,	4019},
    {150    ,	4001},
    {155    ,	3982},
    {160    ,	3964},
    {145    ,	3938},
    {128    ,	3914},
    {120    ,	3895},
    {118    ,	3881},
    {118    ,	3867},
    {125    ,	3857},
    {123    ,	3845},
    {128    ,	3835},
    {133    ,	3825},
    {133    ,	3816},
    {138    ,	3806},
    {128    ,	3788},
    {125    ,	3775},
    {128    ,	3762},
    {128    ,	3748},
    {130    ,	3735},
    {130    ,	3715},
    {133    ,	3695},
    {120    ,	3683},
    {125    ,	3678},
    {135    ,	3675},
    {153    ,	3665},
    {188    ,	3645},
    {210    ,	3617},
    {220    ,	3577},
    {235    ,	3530},
    {250    ,	3479},
    {283    ,	3429},
    {333    ,	3360},
    {500    ,	3252},
    {840    ,	3135},
    {700    ,	3078},
    {588    ,	3034},
    {515    ,	3004},
    {520    ,	3005},
    {430    ,	2971},
    {408    ,	2958},
    {363    ,	2944},
    {345    ,	2931},
    {318    ,	2924},
    {305    ,	2917},
    {285    ,	2913},
    {273    ,	2907},
    {263    ,	2903},
    {255    ,	2900},
    {250    ,	2896},
    {255    ,	2901},
    {240    ,	2890},
    {223    ,	2889},
    {220    ,	2887},
    {218    ,	2886}
};

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};
#endif /* CONFIG_LGE_PM_BATTERY_PROFILE */

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H

