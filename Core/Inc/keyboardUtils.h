#include <stdint.h>    // For uint8_t, uint32_t
#include <stdbool.h>   // For bool type
#include <stddef.h>    // For size_t, NULL
#include <ctype.h>     // For isdigit()

/* ========================================================================= */
/* USB HID Keyboard Specific Definitions                     */
/* ========================================================================= */

// --- Standard HID Keyboard Report Structure (Boot Protocol Compatible) ---
// Matches the simplified HID Report Descriptor (1 modifier, 1 reserved, 6 keycodes)
typedef struct {
    uint8_t MODIFIER; // Bitmap for Ctrl, Shift, Alt, GUI keys
    uint8_t RESERVED; // Reserved, set to 0
    uint8_t KEYCODE1; // Keycode 1 (See HID Usage Tables)
    uint8_t KEYCODE2; // Keycode 2
    uint8_t KEYCODE3; // Keycode 3
    uint8_t KEYCODE4; // Keycode 4
    uint8_t KEYCODE5; // Keycode 5
    uint8_t KEYCODE6; // Keycode 6
} keyboard_report_t; // Should be sizeof() == 8

// --- Modifier Key Masks (Byte 0 of the report) ---
#define KBD_MOD_NONE    (0x00)
#define KBD_MOD_LCTRL   (0x01)
#define KBD_MOD_LSHIFT  (0x02)
#define KBD_MOD_LALT    (0x04)
#define KBD_MOD_LGUI    (0x08) // Windows/Command key
#define KBD_MOD_RCTRL   (0x10)
#define KBD_MOD_RSHIFT  (0x20)
#define KBD_MOD_RALT    (0x40)
#define KBD_MOD_RGUI    (0x80)

// --- Common Keycodes (Partial List - Based on USB HID Usage Tables for Keyboard page) ---
// See http://www.usb.org/developers/hidpage/Hut1_12v2.pdf (Page 53+)
#define KEY_NONE        0x00 // No key pressed
#define KEY_A           0x04
#define KEY_B           0x05
// ... add other letters ...
#define KEY_Z           0x1D
#define KEY_1           0x1E // Keyboard 1 and !
#define KEY_2           0x1F // Keyboard 2 and @
#define KEY_3           0x20 // Keyboard 3 and #
#define KEY_4           0x21
#define KEY_5           0x22
#define KEY_6           0x23
#define KEY_7           0x24
#define KEY_8           0x25
// ... add other numbers ...
#define KEY_9           0x26 // Keyboard 9 and (
#define KEY_0           0x27 // Keyboard 0 and )
#define KEY_ENTER       0x28 // Keyboard Return (ENTER)
#define KEY_ESC         0x29 // Keyboard ESCAPE
#define KEY_BACKSPACE   0x2A // Keyboard DELETE (Backspace)
#define KEY_TAB         0x2B // Keyboard Tab
#define KEY_SPACE       0x2C // Keyboard Spacebar
#define KEY_MINUS       0x2D // Keyboard - and _
#define KEY_EQUAL       0x2E // Keyboard = and +
#define KEY_LEFTBRACE   0x2F // Keyboard [ and {
#define KEY_RIGHTBRACE  0x30 // Keyboard ] and }
#define KEY_BACKSLASH   0x31 // Keyboard \ and |
#define KEY_SEMICOLON   0x33 // Keyboard ; and :
#define KEY_APOSTROPHE  0x34 // Keyboard ' and "
#define KEY_GRAVE       0x35 // Keyboard ` and ~
#define KEY_COMMA       0x36 // Keyboard , and <
#define KEY_DOT         0x37 // Keyboard . and >
#define KEY_SLASH       0x38 // Keyboard / and ?
#define KEY_CAPSLOCK    0x39 // Keyboard Caps Lock
// ... add F-keys, navigation keys, etc. as needed ...
#define KEY_DELETE      0x4C // Keyboard Delete Forward

/* ========================================================================= */
/* Include necessary STM32 HAL and USB Device Headers                   */
/* ========================================================================= */

// !!! Replace stm32xxxx_hal.h with your specific HAL header !!!
// e.g., #include "stm32f4xx_hal.h"
#include "stm32l4xx_hal.h"

// Include the header file for your USB device stack and HID class
#include "usbd_def.h"           // Defines USBD_HandleTypeDef, USBD_StatusTypeDef etc.
#include "usbd_hid_keyboard.h"  // Header for your HID Keyboard class implementation
                                // (Contains USBD_HID_Keyboard_SendReport, USBD_HID_Keyboard_HandleTypeDef, KEYBOARD_HID_IDLE etc.)

/* ========================================================================= */
/* ASCII to HID Mapping Function                        */
/* ========================================================================= */

// Structure to hold the result of the mapping
typedef struct {
    uint8_t keycode;
    uint8_t modifier;
} hid_map_t;

/**
 * @brief Maps an ASCII character to its corresponding USB HID keycode and modifier.
 * Assumes a standard US QWERTY keyboard layout.
 * @param ascii_char The ASCII character to map.
 * @param result Pointer to a hid_map_t structure to store the result.
 * @return true if a mapping exists, false otherwise.
 */
bool ascii_to_hid(char ascii_char, hid_map_t* result) {
    result->keycode = KEY_NONE;       // Default: No key pressed
    result->modifier = KBD_MOD_NONE;  // Default: No modifier

    // Handle letters a-z (lowercase)
    if (ascii_char >= 'a' && ascii_char <= 'z') {
        result->keycode = KEY_A + (ascii_char - 'a');
        result->modifier = KBD_MOD_NONE;
        return true;
    }
    // Handle letters A-Z (uppercase)
    else if (ascii_char >= 'A' && ascii_char <= 'Z') {
        result->keycode = KEY_A + (ascii_char - 'A');
        result->modifier = KBD_MOD_LSHIFT; // Requires Shift
        return true;
    }
    // Handle numbers 1-9
    else if (ascii_char >= '1' && ascii_char <= '9') {
        result->keycode = KEY_1 + (ascii_char - '1');
        result->modifier = KBD_MOD_NONE;
        return true;
    }
    // Handle number 0
    else if (ascii_char == '0') {
        result->keycode = KEY_0;
        result->modifier = KBD_MOD_NONE;
        return true;
    }

    // Handle other printable ASCII characters
    switch (ascii_char) {
        case ' ': result->keycode = KEY_SPACE; result->modifier = KBD_MOD_NONE; return true;
        case '!': result->keycode = KEY_1; result->modifier = KBD_MOD_LSHIFT; return true;
        case '"': result->keycode = KEY_APOSTROPHE; result->modifier = KBD_MOD_LSHIFT; return true;
        case '#': result->keycode = KEY_3; result->modifier = KBD_MOD_LSHIFT; return true;
        case '$': result->keycode = KEY_4; result->modifier = KBD_MOD_LSHIFT; return true;
        case '%': result->keycode = KEY_5; result->modifier = KBD_MOD_LSHIFT; return true;
        case '&': result->keycode = KEY_7; result->modifier = KBD_MOD_LSHIFT; return true;
        case '\'': result->keycode = KEY_APOSTROPHE; result->modifier = KBD_MOD_NONE; return true;
        case '(': result->keycode = KEY_9; result->modifier = KBD_MOD_LSHIFT; return true;
        case ')': result->keycode = KEY_0; result->modifier = KBD_MOD_LSHIFT; return true;
        case '*': result->keycode = KEY_8; result->modifier = KBD_MOD_LSHIFT; return true;
        case '+': result->keycode = KEY_EQUAL; result->modifier = KBD_MOD_LSHIFT; return true;
        case ',': result->keycode = KEY_COMMA; result->modifier = KBD_MOD_NONE; return true;
        case '-': result->keycode = KEY_MINUS; result->modifier = KBD_MOD_NONE; return true;
        case '.': result->keycode = KEY_DOT; result->modifier = KBD_MOD_NONE; return true;
        case '/': result->keycode = KEY_SLASH; result->modifier = KBD_MOD_NONE; return true;
        case ':': result->keycode = KEY_SEMICOLON; result->modifier = KBD_MOD_LSHIFT; return true;
        case ';': result->keycode = KEY_SEMICOLON; result->modifier = KBD_MOD_NONE; return true;
        case '<': result->keycode = KEY_COMMA; result->modifier = KBD_MOD_LSHIFT; return true;
        case '=': result->keycode = KEY_EQUAL; result->modifier = KBD_MOD_NONE; return true;
        case '>': result->keycode = KEY_DOT; result->modifier = KBD_MOD_LSHIFT; return true;
        case '?': result->keycode = KEY_SLASH; result->modifier = KBD_MOD_LSHIFT; return true;
        case '@': result->keycode = KEY_2; result->modifier = KBD_MOD_LSHIFT; return true;
        case '[': result->keycode = KEY_LEFTBRACE; result->modifier = KBD_MOD_NONE; return true;
        case '\\': result->keycode = KEY_BACKSLASH; result->modifier = KBD_MOD_NONE; return true;
        case ']': result->keycode = KEY_RIGHTBRACE; result->modifier = KBD_MOD_NONE; return true;
        case '^': result->keycode = KEY_6; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+6 (US layout)
        case '_': result->keycode = KEY_MINUS; result->modifier = KBD_MOD_LSHIFT; return true;
        case '`': result->keycode = KEY_GRAVE; result->modifier = KBD_MOD_NONE; return true;
        case '{': result->keycode = KEY_LEFTBRACE; result->modifier = KBD_MOD_LSHIFT; return true;
        case '|': result->keycode = KEY_BACKSLASH; result->modifier = KBD_MOD_LSHIFT; return true;
        case '}': result->keycode = KEY_RIGHTBRACE; result->modifier = KBD_MOD_LSHIFT; return true;
        case '~': result->keycode = KEY_GRAVE; result->modifier = KBD_MOD_LSHIFT; return true;

        // Handle common non-printable chars
        case '\n': result->keycode = KEY_ENTER; result->modifier = KBD_MOD_NONE; return true; // Enter key
        case '\t': result->keycode = KEY_TAB; result->modifier = KBD_MOD_NONE; return true; // Tab key
        // Add others like Backspace (KEY_BACKSPACE), Escape (KEY_ESC) if needed

        default:
            // Character not found in the map
            return false;
    }
}

/* ========================================================================= */
/* Function to Send Keystrokes (Type String)                 */
/* ========================================================================= */

/**
 * @brief Sends a sequence of keystrokes over USB HID based on an input string.
 * Handles key press and release for each character.
 * @param pdev Pointer to the USBD_HandleTypeDef structure (USB Device handle).
 * @param keystrokes Null-terminated string containing the characters to type.
 * @param timeout_ms Timeout in milliseconds to wait for HID interface to become idle.
 * @return USBD_OK on success, USBD_FAIL if handle/class invalid, USBD_BUSY if timeout occurs.
 */
USBD_StatusTypeDef SendKeystrokes(USBD_HandleTypeDef *pdev, const char* keystrokes, uint32_t timeout_ms)
{
    hid_map_t mapping;
    keyboard_report_t report = {0}; // Initialize report to all zeros
    USBD_HID_Keyboard_HandleTypeDef *hhid; // Pointer for HID state access
    uint32_t start_time;
    USBD_StatusTypeDef status;

    // --- Basic Validation ---
    if (pdev == NULL || keystrokes == NULL) {
        return USBD_FAIL; // Invalid arguments
    }

    // --- Check USB Device State ---
    // It's crucial the device is configured by the host before sending reports.
    if (pdev->dev_state != USBD_STATE_CONFIGURED) {
        // Optional: Log error or handle differently
        return USBD_FAIL; // Device not configured
    }

    // --- Get HID Class Handle ---
    // This pointer is set by the USB stack when the HID class initializes.
    hhid = (USBD_HID_Keyboard_HandleTypeDef *)pdev->pClassData_HID_Keyboard;
    if (hhid == NULL) {
        // Optional: Log error
        return USBD_FAIL; // HID Class data not available (initialization failed?)
    }

    // --- Iterate Through String ---
    const char* c = keystrokes;
    while (*c != '\0') {
        // --- Map Character to HID Codes ---
        if (ascii_to_hid(*c, &mapping)) {

            // --- Wait for HID Interface to be IDLE before sending PRESS ---
            start_time = HAL_GetTick();
            while (hhid->state != KEYBOARD_HID_IDLE) {
               if (HAL_GetTick() - start_time > timeout_ms) {
                   // Optional: Log timeout error
                   return USBD_BUSY; // Timeout waiting for idle
               }
               // Optional: Add a tiny delay if busy-waiting causes issues
               // HAL_Delay(1);
            }

            // --- Prepare and Send Key PRESS Report ---
            report.MODIFIER = mapping.modifier;
            report.KEYCODE1 = mapping.keycode;
            // Ensure other keys are 0 (important between characters)
            report.KEYCODE2 = 0x00; report.KEYCODE3 = 0x00; report.KEYCODE4 = 0x00;
            report.KEYCODE5 = 0x00; report.KEYCODE6 = 0x00; report.RESERVED = 0x00;

            status = USBD_HID_Keyboard_SendReport(pdev, (uint8_t*)&report, sizeof(report));
            if (status != USBD_OK) {
                // Optional: Log send error (status might be USBD_BUSY or USBD_FAIL)
                return status; // Return the error status from SendReport
            }

            // --- Wait for HID Interface to be IDLE before sending RELEASE ---
            // This ensures the press report transmission has completed.
            start_time = HAL_GetTick();
            while (hhid->state != KEYBOARD_HID_IDLE) {
               if (HAL_GetTick() - start_time > timeout_ms) {
                   // Optional: Log timeout error
                   return USBD_BUSY; // Timeout waiting for idle
               }
               // HAL_Delay(1); // Optional delay
            }

            // --- Prepare and Send Key RELEASE Report (all zeros) ---
            report.MODIFIER = 0x00;
            report.KEYCODE1 = 0x00;
            // Other keycodes are already 0 from previous step

            status = USBD_HID_Keyboard_SendReport(pdev, (uint8_t*)&report, sizeof(report));
            if (status != USBD_OK) {
                // Optional: Log send error
                return status; // Return the error status
            }

            // --- Small Delay Between Characters (optional, can improve compatibility) ---
            HAL_Delay(5); // e.g., 5ms delay

        } else {
            // Handle unsupported character in the input string
            // Option 1: Skip the character
            // Option 2: Return an error
            // return USBD_FAIL; // Example: return error
            // Option 3: Log a warning and continue
        }
        c++; // Move to the next character
    } // End while loop

    // --- Final Wait for Idle (Good Practice) ---
    // Ensures the last key release transmission completes before the function returns.
    start_time = HAL_GetTick();
    while (hhid->state != KEYBOARD_HID_IDLE) {
       if (HAL_GetTick() - start_time > timeout_ms) {
           return USBD_BUSY; // Timeout on final wait
       }
       // HAL_Delay(1); // Optional delay
    }

    return USBD_OK; // String successfully sent
}


/* ========================================================================= */
/* Example Usage in main()                            */
/* ========================================================================= */

// --- Assume these are initialized elsewhere (e.g., by CubeMX) ---
// extern USBD_HandleTypeDef hUsbDevice; // Your global USB device handle declared extern

/*
int main(void) {
    // ... HAL Init, Clock Config, etc. ...

    // Initialize USB Device (including HID Class)
    MX_USB_DEVICE_Init(); // This should initialize hUsbDevice

    // ... Other initializations ...

    while (1) {
        // Example: Send "Hello World!\n" when a button is pressed
        // if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) { // Check button

            // Wait until the device is configured by the host
            // (Alternatively, check state inside SendKeystrokes as implemented above)
            while (hUsbDevice.dev_state != USBD_STATE_CONFIGURED) {
                 HAL_Delay(100);
            }

            // Send the string (using a timeout of 100ms per step)
            USBD_StatusTypeDef send_status = SendKeystrokes(&hUsbDevice, "Hello World!\n", 100);

            if (send_status == USBD_OK) {
                // Success!
            } else if (send_status == USBD_BUSY) {
                // Handle timeout error
            } else {
                // Handle other failure (e.g., device not configured, class error)
            }

            HAL_Delay(500); // Debounce / prevent rapid sending
        // }

        // ... Rest of main loop ...
    }
}
*/
