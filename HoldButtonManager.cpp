#include "HoldButtonManager.h"

HoldButton* HoldButtonManager::_buttons[MAX_HOLD_BUTTONS] = { nullptr };
int HoldButtonManager::_buttonCount = 0;

bool HoldButtonManager::registerButton(HoldButton &btn) {
    if (_buttonCount >= MAX_HOLD_BUTTONS) {
        return false; // no space left
    }
    _buttons[_buttonCount++] = &btn;
    return true;
}

void HoldButtonManager::updateAll() {
    for (int i = 0; i < _buttonCount; i++) {
        if (_buttons[i] != nullptr) {
            _buttons[i]->update();
        }
    }
}
