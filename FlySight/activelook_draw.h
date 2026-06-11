/***************************************************************************
**  FlySight 2 firmware — ActiveLook shared drawing utilities             **
**  Copyright 2025 Bionic Avionics Inc.  (GPL-3.0-or-later)              **
****************************************************************************/

#ifndef ACTIVELOOK_DRAW_H
#define ACTIVELOOK_DRAW_H

#include <stdint.h>
#include <stdbool.h>

/* Physical display dimensions (pixels) */
#define AL_DISPLAY_WIDTH    304
#define AL_DISPLAY_HEIGHT   256

/*
 * Transform logical x (0 = physical left edge of viewer's field of view)
 * to raw display x.  The display IC origin is at the physical bottom-right,
 * so x must be mirrored.  Use this macro for every x coordinate passed to
 * AL_Draw_Text / AL_Draw_Line when specifying layout positions.
 *
 * raw_y = AL_DISPLAY_HEIGHT - logical_y  (done at the call site).
 */
#define AL_TX(x_logical)  ((uint16_t)(AL_DISPLAY_WIDTH - (x_logical)))

/* ---- Low-level BLE draw commands ---------------------------------------- */

/* Erase everything on the display (command 0x01).  Causes a momentary blank.
 * Use only on phase transitions; within a phase use AL_Draw_PadStr instead. */
void AL_Draw_ClearScreen(void);

/* Set the active drawing color (0 = black, 15 = white).
 * Used internally to erase old arrow lines; rarely needed by callers. */
void AL_Draw_SetColor(uint8_t color);

/* Draw a line between two raw-coordinate endpoints. */
void AL_Draw_Line(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

/*
 * Draw a text string at raw (x, y) with the given font and greyscale color.
 * rot = 4 is hardcoded (corrects 180-degree display mounting).
 * font: 1 (smallest) … 5 (largest).  color: 0–15.
 */
void AL_Draw_Text(uint16_t raw_x, uint16_t raw_y, uint8_t font, uint8_t color,
                  const char *str);

/*
 * Copy src into dst and pad the remainder up to max_width with spaces.
 * buf_size must be >= max_width + 1.
 */
void AL_Draw_PadStr(char *dst, const char *src, int max_width, int buf_size);

/* ---- Tracked text field -------------------------------------------------- */

/*
 * Maximum string length (including NUL) stored in AL_TextField_t.
 * Strings longer than this are silently truncated.
 */
#define AL_TEXTFIELD_MAXLEN  24

/*
 * Per-field state for flicker-free text updates.
 * Mirrors the CompetitionMode.cpp technique: draw the old string in colour 0
 * (black / background) to erase it, then draw the new string in colour 15.
 * One instance per on-screen text slot.  Initialise with AL_TEXTFIELD_INIT.
 */
/*
 * Screen-generation counter.  AL_Draw_ClearScreen() increments this; each
 * AL_TextField_t stores the generation when it last drew something.  When
 * field->gen != current generation the field is considered stale (screen was
 * cleared since the last draw) and the erase step is skipped automatically.
 * Call AL_Draw_ScreenGen() to read the current value.
 */
uint32_t AL_Draw_ScreenGen(void);

typedef struct {
    char     prev[AL_TEXTFIELD_MAXLEN]; /* last-drawn string; "" = nothing drawn */
    uint16_t raw_x;
    uint16_t raw_y;
    uint8_t  font;                      /* font used to draw prev (for erase) */
    uint32_t gen;                       /* screen generation when prev was drawn */
} AL_TextField_t;

#define AL_TEXTFIELD_INIT(rx, ry)  { {0}, (rx), (ry), 0, 0 }

/*
 * Erase-then-draw update:
 *   1. If the field is current (same screen generation) and str == prev: no-op.
 *   2. If current and prev != "": redraw prev in colour 0 to erase it.
 *   3. If str != "": draw str in colour 15.
 *   4. Store str and the current generation.
 * After AL_Draw_ClearScreen() the generation advances, so the next call
 * automatically skips the erase step — no manual reset required.
 */
void AL_Draw_TextField(AL_TextField_t *field, uint8_t font, const char *str);

/* ---- Arrow with per-instance erase state --------------------------------- */

/*
 * Persistent state for one arrow polygon.  One instance per on-screen arrow.
 * Initialise with AL_ARROW_STATE_INIT before first use.
 */
typedef struct {
    int16_t prev_rx[7];
    int16_t prev_ry[7];
    int16_t cx;           /* raw display x of arrow center (set by AL_Arrow_Draw) */
    int16_t cy;           /* raw display y of arrow center */
    bool    valid;        /* true if cx/cy and prev_rx/ry contain a drawn arrow */
} AL_ArrowState_t;

#define AL_ARROW_STATE_INIT  { { 0 }, { 0 }, 0, 0, false }

/*
 * Erase the arrow by painting a filled black circle (circf, cmd 0x36) over
 * the stored center, then restoring white.  No-op if the state is not valid.
 * Radius = half the arrow's height (6m = 21 px for scale m = 3.5).
 */
void AL_Arrow_Erase(AL_ArrowState_t *state);

/*
 * Draw a 7-line arrow polygon centred at logical (lx, ly).
 * angle_deg = 0 → arrow points to physical top of display (straight ahead).
 * Vertices are computed in logical (y-up) space and transformed to raw.
 * The previous frame's vertices are stored for the next AL_Arrow_Erase call.
 *
 * Polygon ported from ActiveLook.cpp drawArrow(), scale m = 3.5 px/unit:
 *   vx = {-2m,-2m,-4m,0,4m,2m,2m}  vy = {5m,-2m,-2m,-7m,-2m,-2m,5m}
 *   rotation: radians = (angle_deg + 180) * π / 180
 */
void AL_Arrow_Draw(AL_ArrowState_t *state, int lx, int ly, float angle_deg);

/*
 * Convenience wrapper: draw the arrow then draw a fixed label string below it.
 * label_raw_x / label_raw_y are raw display coordinates for the label.
 * The caller is responsible for padding label with '$' if needed.
 */
void AL_Arrow_DrawWithLabel(AL_ArrowState_t *state, int lx, int ly, float angle_deg,
                             uint16_t label_raw_x, uint16_t label_raw_y,
                             uint8_t label_font, const char *label);

/*
 * Draw a compass-style double-sided arrow centred at logical (lx, ly).
 * The north (forward) half is solid-filled; the south half is an outline only.
 * angle_deg = 0 → north tip points to physical top of display.
 * Same state struct and AL_Arrow_Erase() as AL_Arrow_Draw.
 */
void AL_CompassArrow_Draw(AL_ArrowState_t *state, int lx, int ly, float angle_deg);

#endif /* ACTIVELOOK_DRAW_H */
