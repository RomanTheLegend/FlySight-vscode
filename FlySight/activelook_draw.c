/***************************************************************************
**  FlySight 2 firmware — ActiveLook shared drawing utilities             **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#include "activelook_draw.h"
#include "activelook_client.h"
#include <string.h>
#include <math.h>

/* ---- Internal helpers ---------------------------------------------------- */

static void SendRaw(const uint8_t *data, uint16_t len)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, len);
}

static uint32_t s_screen_gen = 0;

/* ---- Low-level BLE draw commands ----------------------------------------- */

uint32_t AL_Draw_ScreenGen(void) { return s_screen_gen; }

void AL_Draw_ClearScreen(void)
{
    uint8_t pkt[5] = { 0xFF, 0x01, 0x00, 5, 0xAA };
    SendRaw(pkt, sizeof(pkt));
    s_screen_gen++;
}

/* command 0x30: sets active drawing colour (0 = black, 15 = white) */
void AL_Draw_SetColor(uint8_t color)
{
    uint8_t pkt[6] = { 0xFF, 0x30, 0x00, 6, color, 0xAA };
    SendRaw(pkt, sizeof(pkt));
}

/* command 0x32: FF 32 00 0D x1h x1l y1h y1l x2h x2l y2h y2l AA */
void AL_Draw_Line(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    uint8_t pkt[13];
    pkt[0]  = 0xFF; pkt[1]  = 0x32; pkt[2]  = 0x00; pkt[3]  = 13;
    pkt[4]  = (uint8_t)((uint16_t)x1 >> 8);  pkt[5]  = (uint8_t)(x1 & 0xFF);
    pkt[6]  = (uint8_t)((uint16_t)y1 >> 8);  pkt[7]  = (uint8_t)(y1 & 0xFF);
    pkt[8]  = (uint8_t)((uint16_t)x2 >> 8);  pkt[9]  = (uint8_t)(x2 & 0xFF);
    pkt[10] = (uint8_t)((uint16_t)y2 >> 8);  pkt[11] = (uint8_t)(y2 & 0xFF);
    pkt[12] = 0xAA;
    SendRaw(pkt, sizeof(pkt));
}

/*
 * command 0x37: FF 37 00 LEN x_hi x_lo y_hi y_lo rot font color text... AA
 * LEN = 12 + strlen(str).  rot=4 corrects for 180-degree display mounting.
 */
void AL_Draw_Text(uint16_t raw_x, uint16_t raw_y, uint8_t font, uint8_t color,
                  const char *str)
{
    uint8_t n = (uint8_t)strlen(str);
    uint8_t pkt[80];
    uint8_t i = 0;
    pkt[i++] = 0xFF; pkt[i++] = 0x37; pkt[i++] = 0x00;
    pkt[i++] = (uint8_t)(12 + n);
    pkt[i++] = (uint8_t)(raw_x >> 8); pkt[i++] = (uint8_t)(raw_x & 0xFF);
    pkt[i++] = (uint8_t)(raw_y >> 8); pkt[i++] = (uint8_t)(raw_y & 0xFF);
    pkt[i++] = 0x04;   /* rot = 4: 180° rotation for upside-down display */
    pkt[i++] = font;
    pkt[i++] = color;
    memcpy(&pkt[i], str, n); i += n;
    pkt[i++] = 0xAA;
    SendRaw(pkt, i);
}

void AL_Draw_PadStr(char *dst, const char *src, int max_width, int buf_size)
{
    int len = (int)strlen(src);
    if (len > max_width) len = max_width;
    memcpy(dst, src, (size_t)len);
    int k;
    for (k = len; k < max_width && k < buf_size - 1; k++)
        dst[k] = ' ';
    dst[k] = '\0';
}

/* ---- Tracked text field -------------------------------------------------- */

void AL_Draw_TextField(AL_TextField_t *field, uint8_t font, const char *str)
{
    bool current = (field->gen == s_screen_gen);
    if (current && strcmp(field->prev, str) == 0)
        return;
    if (current && field->prev[0] != '\0')
        AL_Draw_Text(field->raw_x, field->raw_y, field->font, 0, field->prev);
    if (str[0] != '\0') {
        field->font = font;
        AL_Draw_Text(field->raw_x, field->raw_y, font, 15, str);
    }
    field->gen = s_screen_gen;
    strncpy(field->prev, str, AL_TEXTFIELD_MAXLEN - 1);
    field->prev[AL_TEXTFIELD_MAXLEN - 1] = '\0';
}

/* ---- Arrow drawing ------------------------------------------------------- */

void AL_Arrow_Erase(AL_ArrowState_t *state)
{
    if (!state->valid)
        return;

    /* Erase by painting a filled black circle over the arrow center, then
     * restoring white.  r = 32 covers the compass arrow tips at 8m = 28 px.
     * color cmd 0x30, circf cmd 0x36 (s16 x, s16 y, u8 r): 6+10+6 = 22 bytes */
    const uint8_t r = 32;
    uint8_t buf[22];
    uint16_t pos = 0;

    buf[pos++] = 0xFF; buf[pos++] = 0x30; buf[pos++] = 0x00;
    buf[pos++] = 6;    buf[pos++] = 0;    buf[pos++] = 0xAA;   /* color = 0 (black) */

    buf[pos++] = 0xFF; buf[pos++] = 0x36; buf[pos++] = 0x00; buf[pos++] = 10;
    buf[pos++] = (uint8_t)((uint16_t)state->cx >> 8); buf[pos++] = (uint8_t)(state->cx & 0xFF);
    buf[pos++] = (uint8_t)((uint16_t)state->cy >> 8); buf[pos++] = (uint8_t)(state->cy & 0xFF);
    buf[pos++] = r;
    buf[pos++] = 0xAA;

    buf[pos++] = 0xFF; buf[pos++] = 0x30; buf[pos++] = 0x00;
    buf[pos++] = 6;    buf[pos++] = 15;   buf[pos++] = 0xAA;   /* color = 15 (white) */

    FS_ActiveLook_Client_WriteWithoutResp(buf, pos);
    state->valid = false;
}

void AL_Arrow_Draw(AL_ArrowState_t *state, int lx, int ly, float angle_deg)
{
    const float m     = 3.5f;
    const float vx[7] = { -2*m, -2*m, -4*m,  0,    4*m,  2*m,  2*m };
    const float vy[7] = {  5*m, -2*m, -2*m, -7*m, -2*m, -2*m,  5*m };

    float rad   = angle_deg * (float)M_PI / 180.0f;
    float cos_r = cosf(rad);
    float sin_r = sinf(rad);

    int16_t rx[7], ry[7];
    for (int i = 0; i < 7; i++) {
        float lxv = (float)lx + vx[i] * cos_r - vy[i] * sin_r;
        float lyv = (float)ly + vx[i] * sin_r + vy[i] * cos_r;
        rx[i] = (int16_t)(AL_DISPLAY_WIDTH  - (int16_t)lxv);
        ry[i] = (int16_t)(AL_DISPLAY_HEIGHT - (int16_t)lyv);
        state->prev_rx[i] = rx[i];
        state->prev_ry[i] = ry[i];
    }

    state->cx = (int16_t)(AL_DISPLAY_WIDTH  - lx);
    state->cy = (int16_t)(AL_DISPLAY_HEIGHT - ly);

    /* polyline cmd 0x38: 8 points (7 vertices + v[0] repeated to close polygon)
     * data = 3 (thickness+res+res) + 8*4 = 35 B; total frame = 4+35+1 = 40 B */
    uint8_t buf[40];
    uint16_t pos = 0;
    buf[pos++] = 0xFF; buf[pos++] = 0x38; buf[pos++] = 0x00; buf[pos++] = 40;
    buf[pos++] = 1;    buf[pos++] = 0;    buf[pos++] = 0;
    for (int i = 0; i <= 7; i++) {
        int j = i % 7;
        buf[pos++] = (uint8_t)((uint16_t)rx[j] >> 8); buf[pos++] = (uint8_t)(rx[j] & 0xFF);
        buf[pos++] = (uint8_t)((uint16_t)ry[j] >> 8); buf[pos++] = (uint8_t)(ry[j] & 0xFF);
    }
    buf[pos++] = 0xAA;
    FS_ActiveLook_Client_WriteWithoutResp(buf, pos);
    state->valid = true;
}

/* Draw a compass-style double-sided arrow: filled top half, outlined bottom half.
 * Same coordinate convention as AL_Arrow_Draw; uses the same state/erase. */
void AL_CompassArrow_Draw(AL_ArrowState_t *state, int lx, int ly, float angle_deg)
{
    const float m = 3.5f;
    const float H = 8.0f * m;   /* center-to-tip distance: 28.0 px */
    const float W = 2.5f * m;   /* center-to-wing distance:  8.75 px */
    const int   NFILL = 14;     /* scan lines at local-y = 0, 2, 4, ..., 26 */

    float rad   = angle_deg * (float)M_PI / 180.0f;
    float cos_r = cosf(rad);
    float sin_r = sinf(rad);

#define _CRX(xl, yl) ((int16_t)(AL_DISPLAY_WIDTH  - ((float)(lx) + (xl)*cos_r - (yl)*sin_r)))
#define _CRY(xl, yl) ((int16_t)(AL_DISPLAY_HEIGHT - ((float)(ly) + (xl)*sin_r + (yl)*cos_r)))
#define _WLINE(x1, y1, x2, y2) do {                                           \
        int16_t _a=(x1),_b=(y1),_c=(x2),_d=(y2);                              \
        buf[pos++]=0xFF; buf[pos++]=0x32; buf[pos++]=0x00; buf[pos++]=13;      \
        buf[pos++]=(uint8_t)((uint16_t)_a>>8); buf[pos++]=(uint8_t)(_a&0xFF); \
        buf[pos++]=(uint8_t)((uint16_t)_b>>8); buf[pos++]=(uint8_t)(_b&0xFF); \
        buf[pos++]=(uint8_t)((uint16_t)_c>>8); buf[pos++]=(uint8_t)(_c&0xFF); \
        buf[pos++]=(uint8_t)((uint16_t)_d>>8); buf[pos++]=(uint8_t)(_d&0xFF); \
        buf[pos++]=0xAA;                                                       \
    } while(0)

    /* 14 fill scan lines + 2 top outline + 2 bottom outline = 18 line commands */
    uint8_t buf[(NFILL + 4) * 13];
    uint16_t pos = 0;

    /* Fill the top (north) half with scan lines in local space.
     * Each line goes from the left to right edge of the triangle at that height.
     * Rotation is applied per-endpoint so the fill works at any heading. */
    for (int k = 0; k < NFILL; k++) {
        float y_l  = (float)(k * 2);
        float half = W * (1.0f - y_l / H);
        _WLINE(_CRX(-half, y_l), _CRY(-half, y_l),
               _CRX( half, y_l), _CRY( half, y_l));
    }

    /* Key vertices in raw display coordinates */
    int16_t top_x = _CRX(0,  H), top_y = _CRY(0,  H);
    int16_t lft_x = _CRX(-W, 0), lft_y = _CRY(-W, 0);
    int16_t rgt_x = _CRX( W, 0), rgt_y = _CRY( W, 0);
    int16_t bot_x = _CRX(0, -H), bot_y = _CRY(0, -H);

    /* Top half outline */
    _WLINE(lft_x, lft_y, top_x, top_y);
    _WLINE(top_x, top_y, rgt_x, rgt_y);

    /* Bottom (south) half outline — hollow */
    _WLINE(lft_x, lft_y, bot_x, bot_y);
    _WLINE(bot_x, bot_y, rgt_x, rgt_y);

#undef _CRX
#undef _CRY
#undef _WLINE

    state->cx = (int16_t)(AL_DISPLAY_WIDTH  - lx);
    state->cy = (int16_t)(AL_DISPLAY_HEIGHT - ly);
    state->valid = true;

    FS_ActiveLook_Client_WriteWithoutResp(buf, pos);
}

void AL_Arrow_DrawWithLabel(AL_ArrowState_t *state, int lx, int ly, float angle_deg,
                             uint16_t label_raw_x, uint16_t label_raw_y,
                             uint8_t label_font, const char *label)
{
    AL_Arrow_Draw(state, lx, ly, angle_deg);
    AL_Draw_Text(label_raw_x, label_raw_y, label_font, 15, label);
}
