#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE
#define C_MAGIC QK_AREP

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  ST_MACRO_0,
  C_MAG_2,
  C_MAG_3
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_LBRC,        KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_RBRC,        
    KC_TAB,         TD(DANCE_0),    KC_M,           KC_L,           TD(DANCE_1),    KC_P,                                           TD(DANCE_2),    C_MAGIC,           KC_O,           KC_U,           KC_Q,           KC_SCLN,        
    KC_LEFT_SHIFT,  KC_S,           KC_T,           KC_R,           KC_D,           KC_Y,                                           KC_F,         KC_N,           KC_A,           KC_E,           KC_I,           MT(MOD_RSFT, KC_QUOTE),
    KC_LEFT_GUI,    MT(MOD_LALT, KC_Z),KC_K,           KC_J,           KC_G,           KC_W,                                        KC_B,           KC_H,           KC_QUOTE,       KC_SCLN,        KC_COMMA,       KC_RIGHT_GUI,   
                                                    LT(2,KC_SPACE), MT(MOD_LSFT, KC_BSPC),                                TD(DANCE_3),   QK_REP 
  ),
  [1] = LAYOUT_voyager(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_GRAVE,       KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_7,           KC_8,           KC_9,           KC_MINUS,       KC_SLASH,       KC_F12,         
    KC_TRANSPARENT, KC_CIRC,        KC_AMPR,        KC_ASTR,        KC_LPRN,        KC_RPRN,                                        KC_4,           KC_5,           KC_6,           KC_PLUS,        KC_ASTR,        KC_BSPC,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_LBRC,        KC_RBRC,        KC_LCBR,        KC_RCBR,                                        KC_1,           KC_2,           KC_3,           KC_DOT,         KC_EQUAL,       KC_ENTER,       
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 TO(0),          KC_0
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 RGB_MODE_FORWARD,KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_AUDIO_MUTE,  KC_TRANSPARENT,                                 RGB_SLD,        LCTL(LSFT(KC_TAB)),KC_UP,          LCTL(KC_TAB),   LALT(LGUI(LCTL(LSFT(KC_I)))),LALT(LGUI(LCTL(LSFT(KC_U)))),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_HYPR,        KC_MEH,         KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_M,           LALT(LGUI(LCTL(LSFT(KC_N)))),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_F13,         KC_F14,         KC_F15,         KC_F16,         KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

const uint16_t PROGMEM combo0[] = { KC_N, KC_A, COMBO_END};
const uint16_t PROGMEM combo1[] = { KC_F, KC_O, COMBO_END};
const uint16_t PROGMEM combo_kj[] = { KC_K, KC_J, COMBO_END};
const uint16_t PROGMEM combo_jg[] = { KC_J, KC_G, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_ESCAPE),
    COMBO(combo1, ST_MACRO_0),
    COMBO(combo_kj, C_MAG_2),
    COMBO(combo_jg, C_MAG_3),
};


extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  t
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152}, {42,211,152} },

    [1] = { {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246}, {191,89,246} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {87,86,157}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {87,86,157}, {87,86,157}, {87,86,157}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LSFT(SS_TAP(X_SCLN)) SS_DELAY(100) SS_TAP(X_W)  SS_DELAY(100) SS_TAP(X_ENTER));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[4];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_V);
        tap_code16(KC_V);
        tap_code16(KC_V);
    }
    if(state->count > 3) {
        tap_code16(KC_V);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_V); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: register_code16(KC_V); register_code16(KC_V); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_V); register_code16(KC_V);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_V); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: unregister_code16(KC_V); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_V); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_C);
        tap_code16(KC_C);
        tap_code16(KC_C);
    }
    if(state->count > 3) {
        tap_code16(KC_C);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_C); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_C)); break;
        case DOUBLE_TAP: register_code16(KC_C); register_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_C); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_C)); break;
        case DOUBLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_C); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_X);
        tap_code16(KC_X);
        tap_code16(KC_X);
    }
    if(state->count > 3) {
        tap_code16(KC_X);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(KC_X); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_X)); break;
        case DOUBLE_TAP: register_code16(KC_X); register_code16(KC_X); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_X); register_code16(KC_X);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(KC_X); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_X)); break;
        case DOUBLE_TAP: unregister_code16(KC_X); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_X); break;
    }
    dance_state[2].step = 0;
}
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: layer_move(1); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: layer_move(1); break;
        case DOUBLE_SINGLE_TAP: layer_move(1); break;
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_HOLD: unregister_code16(KC_LEFT_CTRL); break;
    }
    dance_state[3].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_3_finished, dance_3_reset),
};


// magic keys

bool get_repeat_key_eligible_user(uint16_t keycode, keyrecord_t* record, uint8_t* remembered_mods) {
    switch (keycode) {
        // Ignore Custom Magic Keys
        case C_MAG_2:
        case C_MAG_3:
            return false;
        case KC_A ... KC_Z:
            if ((*remembered_mods & ~(MOD_MASK_SHIFT)) == 0) {
                *remembered_mods &= ~MOD_MASK_SHIFT;
            }
            break;
    }

    return true;
}

uint16_t get_alt_repeat_key_keycode_user(uint16_t keycode, uint8_t mods) {
    switch (keycode) {
        case KC_C:
        case KC_P:
        case KC_D:
        case KC_G:
        case KC_Z: return KC_Y;
        case KC_Y: return KC_P;
        case KC_R: return KC_L;
        case KC_K: return KC_S;
        case KC_L:
        case KC_S: return KC_K;
        case KC_U: return KC_E;
        case KC_E: return KC_U;
        case KC_O: return KC_A;
        case KC_A: return KC_O;
        case KC_DOT:
            if (mods & MOD_MASK_SHIFT) {
                return KC_EQL;
            } else {
                return KC_BSLS;
            }
        case KC_COMM:
            if (mods & MOD_MASK_SHIFT) {
                return KC_EQL;
            } else {
                return MG_SP_BUT;
            }
        case KC_EQL:
        case C_RCTL_MINS:
        case KC_MINS: return KC_RABK;
        case KC_Q: return MG_MLATIV;
        case KC_H: return MG_OA;
        case KC_I: return MG_ON;
        case KC_N: return MG_ION;
        case KC_V: return MG_ER;
        case KC_X: return MG_ES;
        case KC_M: return MG_ENT;
        case KC_T: return MG_MENT;
        case KC_J: return MG_UST;
        case KC_B: return MG_EFORE;
        case KC_W: return MG_HICH;
        case KC_1 ... KC_0: return KC_DOT;
    }

    return MG_THE;
}

bool process_magic_key_2(uint16_t prev_keycode, uint8_t prev_mods) {
    switch (prev_keycode) {
        case KC_B:
            SEND_STRING("ecome");
            return false;
        case KC_F:
            SEND_STRING("ollow");
            return false;
        case KC_N:
            SEND_STRING("eighbor");
            return false;
        case KC_H:
            SEND_STRING("owever");
            return false;
        case KC_U:
            SEND_STRING("pgrade");
            return false;
        case KC_O:
            SEND_STRING("ther");
            return false;
        case KC_A:
            SEND_STRING("lready");
            return false;
        case KC_P:
            SEND_STRING("sych");
            return false;
        case KC_I:
            SEND_STRING("'ll");
            return false;
        case KC_K:
            SEND_STRING("now");
            return false;
        case KC_T:
            SEND_STRING("hough");
            return false;
        case KC_L:
            SEND_STRING("ittle");
            return false;
        case KC_M:
        case KC_R:
            SEND_STRING("ight");
            return false;
        case KC_J:
            SEND_STRING("udge");
            return false;
        case KC_C:
            SEND_STRING("ould");
            return false;
        case KC_D:
            SEND_STRING("evelop");
            return false;
        case KC_G:
            SEND_STRING("eneral");
            return false;
        case KC_W:
            SEND_STRING("here");
            return false;
        case KC_S:
            SEND_STRING("hould");
            return false;
        case KC_DOT:
            SEND_STRING("org");
            return false;
        case KC_COMM:
            SEND_STRING(" however");
            return false;
        default:
            SEND_STRING("'ll");
            return false;
    }
}

bool process_magic_key_3(uint16_t prev_keycode, uint8_t prev_mods) {
    switch (prev_keycode) {
        case KC_B:
            SEND_STRING("etween");
            return false;
        case KC_N:
            SEND_STRING("umber");
            return false;
        case KC_U:
            SEND_STRING("pdate");
            return false;
        case KC_A:
            SEND_STRING("bout");
            return false;
        case KC_W:
            SEND_STRING("orld");
            return false;
        case KC_G:
            SEND_STRING("overn");
            return false;
        case KC_P:
            SEND_STRING("rogram");
            return false;
        case KC_Q:
            SEND_STRING("uestion");
            return false;
        case KC_C:
            SEND_STRING("rowd");
            return false;
        case KC_S:
            SEND_STRING("chool");
            return false;
        case KC_T:
            SEND_STRING("hrough");
            return false;
        case KC_M:
            SEND_STRING("anage");
            return false;
        case KC_O:
            SEND_STRING("xygen");
            return false;
        case KC_I:
            SEND_STRING("'m");
            return false;
        case KC_E:
            SEND_STRING("'re");
            return false;
        case KC_DOT:
            SEND_STRING("com");
            return false;
        case KC_COMM:
            SEND_STRING(" since");
            return false;
        default:
            return false;
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (record->event.pressed) {
        int rep_count = get_repeat_key_count();
        if (rep_count < -1 && keycode != MG_UST) {
            send_char('n');
            return false;
        }
        switch (keycode) {
            case C_MAG_2:
                return process_magic_key_2(get_repeat_key_keycode(), get_repeat_key_mods());
            case C_MAG_3:
                return process_magic_key_3(get_repeat_key_keycode(), get_repeat_key_mods());
            case MK_DUND:
                SEND_STRING(SS_LSFT(SS_TAP(X_4)) SS_DELAY(100) SS_LSFT(SS_TAP(X_MINUS)));
                return false;
            case MG_ENT:
                SEND_STRING("ent");
                return false;
            case MG_MENT:
                SEND_STRING("ment");
                return false;
            case MG_ER:
                SEND_STRING("er");
                return false;
            case MG_ES:
                SEND_STRING("es");
                return false;
            case MG_UST:
                if (rep_count < -1) {
                    SEND_STRING("ment");
                } else {
                    SEND_STRING("ust");
                }
                return false;
            case MG_OA:
                SEND_STRING("oa");
                return false;
            case MG_ON:
                SEND_STRING("on");
                return false;
            case MG_ION:
                SEND_STRING("ion");
                return false;
            case MG_SP_BUT:
                SEND_STRING(" but");
                return false;
            case MG_THE:
                SEND_STRING("the");
                return false;
            case MG_EFORE:
                SEND_STRING("efore");
                return false;
            case MG_HICH:
                SEND_STRING("hich");
                return false;
            case MG_MLATIV:
                SEND_STRING("mlativ");
                return false;
            case MG_QUOT_S:
                SEND_STRING("'s");
                return false;
        }

        if (rep_count > 0) {
            switch (keycode) {
                case KC_BSPC:
                case C_LCTL_BSPC:
                case KC_DQUO:
                case KC_LPRN:
                case KC_SPC:
                case KC_ENT:
                case C_LALT_ENT:
                case C_RSFT_ENT:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("for");
                    return false;
                case KC_I:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("ng");
                    return false;
                case KC_DOT:
                case KC_QUES:
                case KC_EXLM:
                case KC_COLN:
                case KC_SCLN:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    send_char(' ');
                    add_oneshot_mods(MOD_MASK_SHIFT);
                    set_repeat_key_keycode(KC_SPC);
                    return false;
                case KC_COMM:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING(" and");
                    return false;
                case KC_A:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("nd");
                    return false;
                case KC_N:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    send_char('f');
                    return false;
                case KC_B:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("ecause");
                    return false;
                case KC_W:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    SEND_STRING("ould");
                    return false;
                case KC_Y:
                    unregister_weak_mods(MOD_MASK_CSAG);
                    if (rep_count > 2) {
                        SEND_STRING("ll");
                        return false;
                    }
                    if (rep_count > 1) {
                        send_char('\'');
                        return false;
                    }
                    SEND_STRING("ou");
                    return false;
            }
        }
    }
    return true;
}
