#pragma once
//------------------------------------------------------------------------------
// FT8 TIMING CONSTANTS
//------------------------------------------------------------------------------
#define FT8_BROWSE_TIMEOUT_MS 10000u // return to auto after 10s inactivity
#define FT8_SLOT_MS           15000u // all times in milliseconds
#define FT8_RECEIVE_MS        12640u // 79 symbols × 160ms
#define FT8_START_THRESHOLD   100u   // how close to slot boundary to trigger
#define FT8_POPUP_TIME        5000u  // popup message display time
#define FT8_BUTTON_LONG_PRESS 1000u  // long press trigger time

//------------------------------------------------------------------------------
// FT8 TX CONSTANTS
//------------------------------------------------------------------------------
#define FT8_TONE_SPACING   6.25f    // 6.25 Hz
#define FT8_SYMBOL_MS       160u    // 160ms per symbol
#define FT8_DEFAULT_REPORT "-07"    // default signal report
#define FT8_CQ_TX_MIN_HZ    300.0f  // CQ transmit lower limit
#define FT8_CQ_TX_MAX_HZ    2500.0f // CQ transmit upper limit

//------------------------------------------------------------------------------
// FT8 DISPLAY CONSTANTS
//------------------------------------------------------------------------------
#define FT8_DISPLAY_LINES 12u  // lines of decoded messages on screen
#define FT8_DISPLAY_COLS  38u  // truncate display at this column
#define FT8_HISTORY_SIZE  200u // rolling history (~4 slots at 50/slot)

//------------------------------------------------------------------------------
// FT8 QSO CONSTANTS
//------------------------------------------------------------------------------
#define FT8_QSO_NUM_ROWS    6 // row 0=CQ heard, rows 1-5=exchange
#define FT8_DIR_NUM_ROWS    4 // direct call heard, rows 1-4=exchange
#define FT8_QSO_MAX_RETRIES 3 // red after this many missed slots

//------------------------------------------------------------------------------
// FT8 display poition constants
//------------------------------------------------------------------------------
#define FT8_FREQUENCY_X 40
#define FT8_FREQUENCY_Y 21
#define FT8_PULSE_X     0
#define FT8_PULSE_Y     25
#define FT8_STATUS_X    192
#define FT8_STATUS_Y    0
#define FT8_POSITION_X  180
#define FT8_POSITION_Y  8
#define FT8_TXAUDIO_X   192
#define FT8_TXAUDIO_Y   8
#define FT8_CQ_X        180
#define FT8_CQ_Y        0
#define FT8_NEW_X       192
#define FT8_NEW_Y       16
#define FT8_SWR_X       13
#define FT8_SWR_Y       3
#define FT8_PWR_X       13
#define FT8_PWR_Y       13

#define FT8_AGED_COLOUR _rev(0x3186)

//------------------------------------------------------------------------------
// FT8 STATE MACHINES
//------------------------------------------------------------------------------
typedef enum
{
  FT8_STATE_WAITING,     // waiting for next slot boundary
  FT8_STATE_RECEIVING,   // accumulating waterfall (12.64s)
  FT8_STATE_DECODING,    // running decoder (2.36s)
  FT8_STATE_TRANSMITTING // transmitting response
} ft8_state_t;

typedef enum
{
  FT8_UI_AUTO,     // display auto-updates, no cursor
  FT8_UI_BROWSE,   // rotary moved, display frozen, cursor visible
  FT8_UI_SELECTED  // button pressed, message selected, waiting TX slot
} ft8_ui_state_t;

//------------------------------------------------------------------------------
// FT8 DATA STRUCTURES
//------------------------------------------------------------------------------
// One entry in the rolling history ring buffer
typedef struct
{
  char text[FTX_MAX_DISPLAY_LENGTH];
  bool received_in_even_slot; // parity of the slot this was decoded in
  uint8_t slot_number_low;    // low byte of slot number (for age tracking)
} ft8_history_entry_t;

// The currently selected message (for TX response)
typedef struct
{
  char text[FTX_MAX_DISPLAY_LENGTH];
  bool respond_in_even_slot; // TX parity = opposite of received parity
  uint8_t slot_number_low;   // slot number when selected (for age warning)
  bool valid;                // true = something is selected
} ft8_selected_t;

// QSO
typedef enum
{
  FT8_QSO_ROW_PENDING = 0, // dim grey  — not yet reached
  FT8_QSO_ROW_CURRENT,     // inverted  — active step
  FT8_QSO_ROW_WAITING,     // yellow    — TX sent, listening
  FT8_QSO_ROW_RETRY,       // orange    — 2nd attempt
  FT8_QSO_ROW_TIMEOUT,     // red       — 3+ attempts, no reply
  FT8_QSO_ROW_DONE         // green     — completed
} ft8_qso_row_state_t;

typedef struct
{
  char text[FTX_MAX_DISPLAY_LENGTH];
  bool is_tx;
  ft8_qso_row_state_t state;
  uint8_t attempts;  // RX steps: missed slot count
} ft8_qso_row_t;

typedef struct
{
  ft8_qso_row_t rows[FT8_QSO_NUM_ROWS];
  int current_step;
  int num_rows;       // 5 = direct, 6 response to CQ
  char their_call[12];
  char r_report[8];   // "R-10" — built from their step-2 reply
  float audio_freq;   // frquency of their signal
  bool active;
  bool popup_shown;
  bool is_direct;
} ft8_qso_t;

typedef struct
{
  bool active;
  float audio_freq;
  uint8_t attempts;
  bool even_slot; // our TX parity for this CQ session
} ft8_cq_t;

typedef enum
{
  FT8_BTN_IDLE,
  FT8_BTN_SHORT,
  FT8_BTN_LONG
} ft8_btn_t;
