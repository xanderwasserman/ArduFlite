/**
 * HoldButtonManager.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef HOLD_BUTTON_MANAGER_H
#define HOLD_BUTTON_MANAGER_H

#include "src/utils/HoldButton.h"

#define MAX_HOLD_BUTTONS 8  // Adjust to your needs

/**
 * A singleton manager that stores multiple HoldButton objects 
 * and updates them all with a single call to updateAll().
 */
class HoldButtonManager {
public:
    /**
     * Registers a HoldButton reference with the manager.
     * Must be done before using updateAll().
     */
    static bool registerButton(HoldButton &btn);

    /**
     * Calls update() on all registered buttons.
     */
    static void updateAll();

private:
    // We store references to up to MAX_HOLD_BUTTONS
    static HoldButton* _buttons[MAX_HOLD_BUTTONS];
    static int _buttonCount;
};

#endif // HOLD_BUTTON_MANAGER_H
