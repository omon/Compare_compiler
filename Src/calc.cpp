
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include "../DSP/Include/arm_math.h"

#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef hlpuart1;

float i = 0;
constexpr float m_pi = 3.141592653589793f;
constexpr float m_pi_2 = m_pi / 2.f;
extern RNG_HandleTypeDef hrng;

static const size_t buff_size = 1000; // calculate continous

static volatile float rslt_buff[buff_size];
static volatile float cmath_buff[buff_size];
static volatile float ix_buff[buff_size];
static volatile float ix_buff2[buff_size];

extern int get_time_ms();
extern uint64_t get_usec();

char send[128];

static void output_elapse(const char name[], const float start_usec, const size_t times)
{
    auto now = get_usec();
    auto elapse = now - start_usec;
    auto usec = (float)elapse / static_cast<float>(times);
    snprintf(send, sizeof(send), "%s : %.2fusec  (%.2fmsec)\n", name, usec, elapse / 1000.f);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)send, strlen(send), 500);
}

static void output_error(const volatile float cmath[], const volatile float rslt[])
{
    float worst = 0;
    for (size_t i = 0; i < buff_size; i++)
    {
        auto tmp = fabs(cmath[i] - rslt[i]);
        worst = std::max(tmp, worst);
    }
    snprintf(send, sizeof(send), "\terror : %f\n", worst);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)send, strlen(send), 500);
}
float ATAN_LUT[256] = {0.0000000000, 0.0039215485, 0.0078429764, 0.0117641631, 0.0156849881, 0.0196053309,
                       0.0235250710, 0.0274440882, 0.0313622624, 0.0352794736, 0.0391956019, 0.0431105278,
                       0.0470241318, 0.0509362949, 0.0548468980, 0.0587558227, 0.0626629506, 0.0665681638,
                       0.0704713446, 0.0743723758, 0.0782711405, 0.0821675224, 0.0860614053, 0.0899526737,
                       0.0938412126, 0.0977269074, 0.1016096438, 0.1054893085, 0.1093657884, 0.1132389710,
                       0.1171087446, 0.1209749978, 0.1248376255, 0.1286965013, 0.1325515323, 0.1364026044,
                       0.1402496096, 0.1440924408, 0.1479309912, 0.1517651553, 0.1555948280, 0.1594199049,
                       0.1632402828, 0.1670558588, 0.1708665312, 0.1746721990, 0.1784727620, 0.1822681208,
                       0.1860581771, 0.1898428334, 0.1936219929, 0.1973955598, 0.2011634395, 0.2049255380,
                       0.2086817623, 0.2124320205, 0.2161762215, 0.2199142752, 0.2236460927, 0.2273715857,
                       0.2310906672, 0.2348032511, 0.2385092525, 0.2422085871, 0.2459011721, 0.2495869254,
                       0.2532657662, 0.2569376146, 0.2606023917, 0.2642600199, 0.2679104224, 0.2715535237,
                       0.2751892491, 0.2788175253, 0.2824382800, 0.2860514417, 0.2896569404, 0.2932547070,
                       0.2968446734, 0.3004267728, 0.3040009393, 0.3075671084, 0.3111252164, 0.3146752558,
                       0.3182170002, 0.3217505544, 0.3252758042, 0.3287926915, 0.3323011594, 0.3358011520,
                       0.3392926145, 0.3427754932, 0.3462497357, 0.3497152904, 0.3531721069, 0.3566201360,
                       0.3600593294, 0.3634896400, 0.3669110217, 0.3703234297, 0.3737268255, 0.3771211497,
                       0.3805063771, 0.3838824615, 0.3872493632, 0.3906070437, 0.3939554653, 0.3972945915,
                       0.4006243869, 0.4039448169, 0.4072558481, 0.4105574480, 0.4138495853, 0.4171322295,
                       0.4204053512, 0.4236689219, 0.4269229141, 0.4301673014, 0.4334020581, 0.4366271598,
                       0.4398425828, 0.4430483044, 0.4462443029, 0.4494305575, 0.4526070482, 0.4557737560,
                       0.4589306629, 0.4620777516, 0.4652150058, 0.4683424102, 0.4714599501, 0.4745676117,
                       0.4776653824, 0.4807532499, 0.4838312032, 0.4868992318, 0.4899573263, 0.4930054778,
                       0.4960436784, 0.4990719209, 0.5020901990, 0.5050985071, 0.5080968402, 0.5110851942,
                       0.5140635659, 0.5170319525, 0.5199903521, 0.5229387636, 0.5258771863, 0.5288056206,
                       0.5317240673, 0.5346325278, 0.5375310045, 0.5404195003, 0.5432980185, 0.5461665634,
                       0.5490251398, 0.5518737530, 0.5547124091, 0.5575411147, 0.5603598769, 0.5631687036,
                       0.5659676030, 0.5687565842, 0.5715356566, 0.5743048302, 0.5770641155, 0.5798135236,
                       0.5825530662, 0.5852827553, 0.5880026035, 0.5907126240, 0.5934128303, 0.5961032364,
                       0.5987838570, 0.6014547069, 0.6041158015, 0.6067671569, 0.6094087892, 0.6120407151,
                       0.6146629519, 0.6172755171, 0.6198784285, 0.6224717045, 0.6250553640, 0.6276294258,
                       0.6301939095, 0.6327488350, 0.6352942223, 0.6378300921, 0.6403564651, 0.6428733625,
                       0.6453808058, 0.6478788169, 0.6503674179, 0.6528466311, 0.6553164793, 0.6577769856,
                       0.6602281731, 0.6626700655, 0.6651026865, 0.6675260602, 0.6699402110, 0.6723451634,
                       0.6747409422, 0.6771275725, 0.6795050796, 0.6818734889, 0.6842328261, 0.6865831172,
                       0.6889243882, 0.6912566655, 0.6935799756, 0.6958943450, 0.6981998008, 0.7004963699,
                       0.7027840796, 0.7050629571, 0.7073330300, 0.7095943260, 0.7118468729, 0.7140906986,
                       0.7163258312, 0.7185522990, 0.7207701302, 0.7229793534, 0.7251799971, 0.7273720900,
                       0.7295556609, 0.7317307387, 0.7338973524, 0.7360555311, 0.7382053040, 0.7403467003,
                       0.7424797493, 0.7446044805, 0.7467209234, 0.7488291075, 0.7509290624, 0.7530208178,
                       0.7551044035, 0.7571798492, 0.7592471847, 0.7613064400, 0.7633576449, 0.7654008294,
                       0.7674360235, 0.7694632573, 0.7714825607, 0.7734939638, 0.7754974968, 0.7774931897,
                       0.7794810727, 0.7814611759, 0.7834335294, 0.7853981634};
float fast_atan2(const float y, const float x)
{
    float absx, absy;
    absy = fabs(y);
    absx = fabs(x);
    int octant = ((x < 0) << 2) + ((y < 0) << 1) + (absx <= absy);
    switch (octant)
    {
    case 0:
    {
        if (x == 0 && y == 0)
            return 0;
        return ATAN_LUT[static_cast<int>(255.f * absy / absx)]; //1st octant
        break;
    }
    case 1:
    {
        if (x == 0 && y == 0)
            return 0.0;
        return m_pi_2 - ATAN_LUT[static_cast<int>(255.f * absx / absy)]; //2nd octant
        break;
    }
    case 2:
    {
        return -ATAN_LUT[static_cast<int>(255.f * absy / absx)]; //8th octant
        break;
    }
    case 3:
    {
        return -m_pi_2 + ATAN_LUT[static_cast<int>(255.f * absx / absy)]; //7th octant
        break;
    }
    case 4:
    {
        return m_pi - ATAN_LUT[static_cast<int>(255.f * absy / absx)]; //4th octant
    }
    case 5:
    {
        return m_pi_2 + ATAN_LUT[static_cast<int>(255.f * absx / absy)]; //3rd octant
        break;
    }
    case 6:
    {
        return -m_pi + ATAN_LUT[static_cast<int>(255.f * absy / absx)]; //5th octant
        break;
    }
    case 7:
    {
        return -m_pi_2 - ATAN_LUT[static_cast<int>(255.f * absx / absy)]; //6th octant
        break;
    }
    default:
        return 0.0;
    }
}

void calc()
{

    // == output compiler name ==============================================
#ifdef __GNUC__
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)"GCC : ===\n", 10, 500);
#else
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)"IAR : ===\n", 10, 500);
#endif /* __GNUC__ */

    // == check elapse time ==============================================
    auto start = get_usec();
    HAL_Delay(500);
    output_elapse("check 500msec", start, 1);

    for (size_t i = 0; i < buff_size; i++)
    {
        uint32_t ix;
        HAL_RNG_GenerateRandomNumber(&hrng, &ix);
        ix_buff[i] = (static_cast<float>(ix) / static_cast<float>(0x80000000) - 1.f) * 4095.f;

        HAL_RNG_GenerateRandomNumber(&hrng, &ix);
        ix_buff2[i] = (static_cast<float>(ix) / static_cast<float>(0x80000000) - 1.f) * 4095.f;
    }

    // == sin =============================================================
    start = get_usec();
    for (size_t i = 0; i < buff_size; i++)
    {
        cmath_buff[i] = sinf(ix_buff[i]);
    }
    output_elapse("sin", start, buff_size);

    // == CMSIS sin =============================================================
    start = get_usec();
    for (size_t i = 0; i < buff_size; i++)
    {
        rslt_buff[i] = arm_sin_f32(ix_buff[i]);
    }
    output_elapse("cmsis sin", start, buff_size);
    output_error(cmath_buff, rslt_buff);

    // == atan2 =============================================================
    start = get_usec();
    for (size_t i = 0; i < buff_size; i++)
    {
        cmath_buff[i] = atan2f(ix_buff[i], ix_buff2[i]);
    }
    output_elapse("atan2", start, buff_size);

    // == fat atan2 =============================================================
    start = get_usec();
    for (size_t i = 0; i < buff_size; i++)
    {
        rslt_buff[i] = fast_atan2(ix_buff[i], ix_buff2[i]);
    }
    output_elapse("fast atan2", start, buff_size);
    output_error(cmath_buff, rslt_buff);
}
