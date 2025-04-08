/**
 * MultiTapButtonManager.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef MULTI_TAP_BUTTON_MANAGER_H
#define MULTI_TAP_BUTTON_MANAGER_H

#include "src/utils/MultiTapButton.h"

#define MAX_MULTI_TAP_BUTTONS 8  // Adjust to your needs

/**
 * A singleton manager that stores multiple MultiTapButton objects 
 * and updates them all with a single call to updateAll().
 */
class MultiTapButtonManager {
public:
    /**
     * Registers a HoldButton reference with the manager.
     * Must be done before using updateAll().
     */
    static bool registerButton(MultiTapButton &btn);

    /**
     * Calls update() on all registered buttons.
     */
    static void updateAll();

private:
    // We store references to up to MAX_HOLD_BUTTONS
    static MultiTapButton* _buttons[MAX_MULTI_TAP_BUTTONS];
    static int _buttonCount;
};

#endif // MULTI_TAP_BUTTON_MANAGER_H
