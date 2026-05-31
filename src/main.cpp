#include <gui/WinMain.h>
#include <td/StringConverter.h>
#include "Application.h"

#ifdef _WIN32
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include <cstdio>

static void enableConsoleOutput() {
#if defined(_DEBUG)
    if (AttachConsole(ATTACH_PARENT_PROCESS) || AllocConsole()) {
        FILE* fp = nullptr;
        freopen_s(&fp, "CONOUT$", "w", stdout);
        freopen_s(&fp, "CONOUT$", "w", stderr);
        freopen_s(&fp, "CONIN$", "r", stdin);
        setvbuf(stdout, nullptr, _IONBF, 0);
        setvbuf(stderr, nullptr, _IONBF, 0);
        SetConsoleOutputCP(CP_UTF8);
    }
#endif
}
#endif

int main(int argc, const char* argv[]) {
#ifdef _WIN32
    enableConsoleOutput();
#endif
    Application app(argc, argv);
    auto appProperties = app.getProperties();
    td::String trLang = appProperties->getValue("translation", "EN");
    app.init(trLang);
    return app.run();
}
