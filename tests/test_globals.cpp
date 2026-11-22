// test_globals.cpp - the globals main.cpp normally provides, for the test runner.
#include "App.h"

App g;
void showMessage(const std::string& m, float s) { g.message = m; g.messageTime = s; }
void playBeep() {}
