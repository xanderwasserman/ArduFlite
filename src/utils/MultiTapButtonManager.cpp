/**
 * MultiTapButtonManager.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/MultiTapButtonManager.h"

MultiTapButton* MultiTapButtonManager::_buttons[MAX_MULTI_TAP_BUTTONS] = { nullptr };
int MultiTapButtonManager::_buttonCount = 0;

bool MultiTapButtonManager::registerButton(MultiTapButton &btn) {
    if (_buttonCount >= MAX_MULTI_TAP_BUTTONS) {
        return false; // no space left
    }
    _buttons[_buttonCount++] = &btn;
    return true;
}

void MultiTapButtonManager::updateAll() {
    for (int i = 0; i < _buttonCount; i++) {
        if (_buttons[i] != nullptr) {
            _buttons[i]->update();
        }
    }
}
