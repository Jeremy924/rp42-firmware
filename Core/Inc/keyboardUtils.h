/*
 * KeyboardUtils.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Jerem
 */

#ifndef INC_KEYBOARDUTILS_H_
#define INC_KEYBOARDUTILS_H_

#define KBD_MOD_NONE    (0x00)
#define KBD_MOD_LSHIFT  (0x02)
// Add other modifiers if needed (LCtrl=0x01, LAlt=0x04, LGui=0x08, etc.)

// Structure to hold the result
typedef struct {
    uint8_t keycode;
    uint8_t modifier;
} hid_map_t;

// Function to map ASCII char to HID keycode and modifier (US QWERTY)
// Returns true if mapping exists, false otherwise.
bool ascii_to_hid(char ascii_char, hid_map_t* result) {
    result->keycode = 0x00; // Default: No key pressed
    result->modifier = KBD_MOD_NONE; // Default: No modifier

    // Handle letters a-z and A-Z
    if (ascii_char >= 'a' && ascii_char <= 'z') {
        result->keycode = 0x04 + (ascii_char - 'a');
        result->modifier = KBD_MOD_NONE;
        return true;
    } else if (ascii_char >= 'A' && ascii_char <= 'Z') {
        result->keycode = 0x04 + (ascii_char - 'A');
        result->modifier = KBD_MOD_LSHIFT;
        return true;
    }

    // Handle numbers 1-9 and 0
    if (ascii_char >= '1' && ascii_char <= '9') {
        result->keycode = 0x1E + (ascii_char - '1');
        result->modifier = KBD_MOD_NONE;
        return true;
    } else if (ascii_char == '0') {
        result->keycode = 0x27; // Keyboard 0
        result->modifier = KBD_MOD_NONE;
        return true;
    }

    // Handle other characters using a switch case for clarity
    switch (ascii_char) {
        case ' ': result->keycode = 0x2C; result->modifier = KBD_MOD_NONE; return true; // Space
        case '!': result->keycode = 0x1E; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+1
        case '"': result->keycode = 0x34; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+'
        case '#': result->keycode = 0x20; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+3
        case '$': result->keycode = 0x21; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+4
        case '%': result->keycode = 0x22; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+5
        case '&': result->keycode = 0x24; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+7
        case '\'': result->keycode = 0x34; result->modifier = KBD_MOD_NONE; return true; // '
        case '(': result->keycode = 0x26; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+9
        case ')': result->keycode = 0x27; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+0
        case '*': result->keycode = 0x25; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+8
        case '+': result->keycode = 0x2E; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+=
        case ',': result->keycode = 0x36; result->modifier = KBD_MOD_NONE; return true; // ,
        case '-': result->keycode = 0x2D; result->modifier = KBD_MOD_NONE; return true; // -
        case '.': result->keycode = 0x37; result->modifier = KBD_MOD_NONE; return true; // .
        case '/': result->keycode = 0x38; result->modifier = KBD_MOD_NONE; return true; // /
        case ':': result->keycode = 0x33; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+;
        case ';': result->keycode = 0x33; result->modifier = KBD_MOD_NONE; return true; // ;
        case '<': result->keycode = 0x36; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+,
        case '=': result->keycode = 0x2E; result->modifier = KBD_MOD_NONE; return true; // =
        case '>': result->keycode = 0x37; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+.
        case '?': result->keycode = 0x38; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+/
        case '@': result->keycode = 0x1F; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+2
        case '[': result->keycode = 0x2F; result->modifier = KBD_MOD_NONE; return true; // [
        case '\\': result->keycode = 0x31; result->modifier = KBD_MOD_NONE; return true; // \
        case ']': result->keycode = 0x30; result->modifier = KBD_MOD_NONE; return true; // ]
        case '^': result->keycode = 0x23; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+6
        case '_': result->keycode = 0x2D; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+-
        case '`': result->keycode = 0x35; result->modifier = KBD_MOD_NONE; return true; // `
        case '{': result->keycode = 0x2F; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+[
        case '|': result->keycode = 0x31; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+\
        case '}': result->keycode = 0x30; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+]
        case '~': result->keycode = 0x35; result->modifier = KBD_MOD_LSHIFT; return true; // Shift+`

        // Handle common non-printable chars if needed
        case '\n': result->keycode = 0x28; result->modifier = KBD_MOD_NONE; return true; // Enter
        case '\t': result->keycode = 0x2B; result->modifier = KBD_MOD_NONE; return true; // Tab
        // Add Backspace (0x2A), Esc (0x29) etc. if required

        default:
            // Character not found in the map
            return false;
    }
}

// Assuming your keyboard report structure:
typedef struct {
    uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t KEYCODE1;
    uint8_t KEYCODE2;
    uint8_t KEYCODE3;
    uint8_t KEYCODE4;
    uint8_t KEYCODE5;
    uint8_t KEYCODE6;
} keyboard_report_t;

// Corrected function signature
USBD_StatusTypeDef SendKeystrokes(USBD_HandleTypeDef *pdev, const char* keystrokes)
{
    hid_map_t mapping;
    keyboard_report_t report = {0};
    USBD_HID_Keyboard_HandleTypeDef *hhid; // Pointer for HID state access

    // Check device state and get HID handle ONCE if possible (or check inside loop)
    if (pdev->dev_state != USBD_STATE_CONFIGURED) {
        return USBD_FAIL; // Or handle error appropriately
    }
    hhid = (USBD_HID_Keyboard_HandleTypeDef *)pdev->pClassData_HID_Keyboard;
    if (hhid == NULL) {
         return USBD_FAIL; // HID Class not ready
    }


    const char* c = keystrokes; // Use const char*

    while (*c != '\0') {
        if (ascii_to_hid(*c, &mapping)) {

            // --- Wait for IDLE before PRESS ---
            uint32_t start_time = HAL_GetTick();
            while (hhid->state != KEYBOARD_HID_IDLE) {
               if (HAL_GetTick() - start_time > 100) return USBD_BUSY; // Prevent infinite loop
               // You might need HAL_Delay(1) here if busy-waiting starves other tasks
            }

            // Prepare the key press report
            report.MODIFIER = mapping.modifier;
            report.KEYCODE1 = mapping.keycode;
            // Ensure others are 0 for safety between loops
            report.KEYCODE2 = 0x00; report.KEYCODE3 = 0x00; report.KEYCODE4 = 0x00;
            report.KEYCODE5 = 0x00; report.KEYCODE6 = 0x00; report.RESERVED = 0x00;


            // Send key press (Check return status!)
            USBD_StatusTypeDef status = USBD_HID_Keyboard_SendReport(pdev, (uint8_t*)&report, sizeof(report));
            if (status != USBD_OK) {
                // Handle error (e.g., return status;)
                return status;
            }

            // --- Wait for IDLE before RELEASE ---
            start_time = HAL_GetTick();
            while (hhid->state != KEYBOARD_HID_IDLE) {
               if (HAL_GetTick() - start_time > 100) return USBD_BUSY;
            }

            // Prepare the key release report (all zeros)
            report.MODIFIER = 0x00;
            report.KEYCODE1 = 0x00;
            // Other KEYCODEs already 0

            // Send key release (Check return status!)
            status = USBD_HID_Keyboard_SendReport(pdev, (uint8_t*)&report, sizeof(report));
             if (status != USBD_OK) {
                // Handle error
                return status;
             }
        } else {
            // Handle unsupported character - maybe skip or return error?
            // return USBD_FAIL; // Original behavior
        }
        c++;
    }

    // Final wait for idle ensures last release completes before function returns (optional but good practice)
    uint32_t start_time = HAL_GetTick();
    while (hhid->state != KEYBOARD_HID_IDLE) {
       if (HAL_GetTick() - start_time > 100) return USBD_BUSY;
    }

    return USBD_OK;
}

#endif /* INC_KEYBOARDUTILS_H_ */
