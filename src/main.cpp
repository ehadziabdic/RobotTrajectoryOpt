#include <gui/WinMain.h>
#include <td/StringConverter.h>
#include "Application.h"

int main(int argc, const char* argv[]) {
    Application app(argc, argv);
    auto appProperties = app.getProperties();
    td::String trLang = appProperties->getValue("translation", "EN");
    app.init(trLang);
    return app.run();
}
