/**
 * test_production_contracts.cpp - Host-side production safety contracts.
 *
 * These tests pin down firmware-only safety guards that are hard to instantiate
 * in the current host suite because they depend on Arduino WebServer, LittleFS,
 * FreeRTOS, or hardware-backed singletons.
 */
#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <string>

namespace {

std::string readRepoFile(const std::string& relPath)
{
    std::ifstream in(std::string(ARDUFLITE_ROOT) + "/" + relPath);
    if (!in)
    {
        ADD_FAILURE() << "Could not read " << relPath;
        return {};
    }

    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void expectContains(const std::string& text, const std::string& needle)
{
    EXPECT_NE(text.find(needle), std::string::npos) << "Missing: " << needle;
}

void expectNotContains(const std::string& text, const std::string& needle)
{
    EXPECT_EQ(text.find(needle), std::string::npos) << "Unexpected: " << needle;
}

std::string sliceBetween(const std::string& text, const std::string& begin, const std::string& end)
{
    const size_t start = text.find(begin);
    if (start == std::string::npos)
    {
        ADD_FAILURE() << "Missing begin marker: " << begin;
        return {};
    }

    const size_t finish = text.find(end, start + begin.size());
    if (finish == std::string::npos)
    {
        ADD_FAILURE() << "Missing end marker after: " << begin;
        return text.substr(start);
    }

    return text.substr(start, finish - start);
}

} // namespace

TEST(ProductionContracts, WebMutationsRequireSessionTokenAndGroundLocks)
{
    const std::string web = readRepoFile("src/web/ArduFliteWebServer.cpp");

    expectContains(web, "_server->on(\"/api/session\"");
    expectContains(web, "_server->collectHeaders(WEB_HEADER_KEYS, WEB_HEADER_KEY_COUNT)");
    expectContains(web, "_server->header(\"X-ArduFlite-Token\")");

    const std::string configSet = sliceBetween(
        web,
        "void ArduFliteWebServer::handleConfigSet()",
        "void ArduFliteWebServer::handleConfigReset()");
    expectContains(configSet, "if (!isMutationAuthorized()) return;");
    expectContains(configSet, "_controller->isArmed()");
    expectContains(configSet, "sendError(423");

    const std::string configReset = sliceBetween(
        web,
        "void ArduFliteWebServer::handleConfigReset()",
        "void ArduFliteWebServer::handleConfigExport()");
    expectContains(configReset, "if (!isMutationAuthorized()) return;");
    expectContains(configReset, "_controller->isArmed()");

    const std::string configImport = sliceBetween(
        web,
        "void ArduFliteWebServer::handleConfigImport()",
        "void ArduFliteWebServer::handleConfigReboot()");
    expectContains(configImport, "if (!isMutationAuthorized()) return;");
    expectContains(configImport, "_controller->isArmed()");
    expectContains(configImport, "json.length() > 16384");

    const std::string reboot = sliceBetween(
        web,
        "void ArduFliteWebServer::handleConfigReboot()",
        "void ArduFliteWebServer::handleSystemStatus()");
    expectContains(reboot, "if (!isMutationAuthorized()) return;");
    expectContains(reboot, "_controller->isArmed()");

    const std::string calibrate = sliceBetween(
        web,
        "void ArduFliteWebServer::handleCalibrate()",
        "void ArduFliteWebServer::handleFlashList()");
    expectContains(calibrate, "if (!isMutationAuthorized()) return;");
    expectContains(calibrate, "_controller->isArmed()");

    const std::string flashDelete = sliceBetween(
        web,
        "void ArduFliteWebServer::handleFlashDelete()",
        "void ArduFliteWebServer::handleNotFound()");
    expectContains(flashDelete, "if (!isMutationAuthorized()) return;");
    expectContains(flashDelete, "_controller->isArmed()");
    expectContains(flashDelete, "_flashTelemetry->isLogging()");
    expectContains(flashDelete, "!isLogFilename(name)");
}

TEST(ProductionContracts, WebSecretsFlashAndCaptiveDnsStayProtected)
{
    const std::string web = readRepoFile("src/web/ArduFliteWebServer.cpp");
    const std::string wifiH = readRepoFile("src/web/WiFiManager.h");
    const std::string wifiCpp = readRepoFile("src/web/WiFiManager.cpp");
    const std::string configRegistry = readRepoFile("src/utils/ConfigRegistry.cpp");
    const std::string appJs = readRepoFile("tools/web_ui/src/app.js");
    const std::string compressPy = readRepoFile("tools/web_ui/compress.py");

    expectContains(web, "if (std::string(p.key) == CONFIG_KEY_WEB_AP_PASS) continue;");
    expectContains(web, "sendError(403, \"Forbidden: sensitive key\")");
    expectContains(web, "remove(CONFIG_KEY_WEB_AP_PASS)");

    const std::string flashList = sliceBetween(
        web,
        "void ArduFliteWebServer::handleFlashList()",
        "void ArduFliteWebServer::handleFlashGet()");
    expectContains(flashList, "_controller->isArmed()");
    expectContains(flashList, "_flashTelemetry->isLogging()");
    expectContains(flashList, "isLogFilename(name)");

    const std::string flashGet = sliceBetween(
        web,
        "void ArduFliteWebServer::handleFlashGet()",
        "void ArduFliteWebServer::handleFlashDelete()");
    expectContains(flashGet, "_controller->isArmed()");
    expectContains(flashGet, "_flashTelemetry->isLogging()");
    expectContains(flashGet, "!isLogFilename(name)");
    expectContains(flashGet, "_server->client().connected()");
    expectContains(flashGet, "vTaskDelay(pdMS_TO_TICKS(1))");

    expectContains(wifiH, "void processDns()");
    expectContains(wifiCpp, "_dnsServer.processNextRequest()");
    expectContains(web, "WiFiManager::instance().processDns()");
    expectContains(wifiCpp, "_password.length() < 8 || _password == \"arduflite\"");
    expectContains(wifiCpp, "_password = _ssid");
    expectContains(configRegistry, "value.length() < 8 || value == \"arduflite\"");

    expectContains(appJs, "session: '/api/session'");
    expectContains(appJs, "'X-ArduFlite-Token': sessionToken");
    expectContains(appJs, "await fetchOk(`${API.flash}/${name}`");
    expectContains(compressPy, "html_text.replace(css_tag");
    expectContains(compressPy, "<style>");
    expectContains(compressPy, "html_text.replace(js_tag");
    expectContains(compressPy, "<script>");
    expectNotContains(web, "handleCSS");
    expectNotContains(web, "handleJS");
}

TEST(ProductionContracts, FlashTelemetryResetAndDeleteAreSafe)
{
    const std::string header = readRepoFile("src/telemetry/flash/ArduFliteFlashTelemetry.h");
    const std::string cpp = readRepoFile("src/telemetry/flash/ArduFliteFlashTelemetry.cpp");

    expectContains(header, "bool stopLogging()");
    expectContains(header, "if (!stopLogging())");
    expectContains(header, "Flash reset aborted");
    expectContains(cpp, "bool ArduFliteFlashTelemetry::stopLogging()");
    expectContains(cpp, "return false;");
    expectContains(cpp, "return true;");
    expectContains(cpp, "_isLogging && strcmp(fn, _currentFilename) == 0");
    expectContains(cpp, "rowTruncated");
    expectContains(cpp, "!rowTruncated && self->_fileMutex");
    expectContains(cpp, "log index space exhausted");
}

TEST(ProductionContracts, CliInputParsingAndDiagnosticsStayFailSafe)
{
    const std::string cliUtils = readRepoFile("src/cli/CLICommandUtils.cpp");
    const std::string cliContext = readRepoFile("src/cli/CLICommandContext.cpp");
    const std::string cliConfig = readRepoFile("src/cli/CLICommandsConfig.cpp");
    const std::string cliSystem = readRepoFile("src/cli/CLICommandsSystem.cpp");
    const std::string cliFlash = readRepoFile("src/cli/CLICommandsFlash.cpp");
    const std::string cliTelemetry = readRepoFile("src/cli/CLICommandsTelemetry.cpp");
    const std::string cliTests = readRepoFile("src/cli/CLICommandsTests.cpp");
    const std::string cliAll =
        cliUtils + cliContext + cliConfig + cliSystem + cliFlash + cliTelemetry + cliTests;

    expectContains(cliUtils, "parseIntStrict");
    expectContains(cliUtils, "parseFloatStrict");
    expectContains(cliUtils, "parseBoolStrict");
    expectContains(cliContext, "cliController->isArmed()");
    expectContains(cliContext, "cliIMU->getFlightState() == INFLIGHT");
    expectContains(cliSystem, "rejectUnsafeGroundCommand(\"reset\")");
    expectContains(cliSystem, "rejectUnsafeGroundCommand(\"calibrate\")");
    expectContains(cliConfig, "rejectUnsafeGroundCommand(\"change configuration\")");
    expectContains(cliFlash, "rejectUnsafeGroundCommand(\"delete flash logs\")");
    expectContains(cliTests, "test loops");
    expectContains(cliTests, "runControlLoopTest_dtComputation(*controller)");
    expectNotContains(cliAll, "CMD_SET_SETPOINT");
    expectNotContains(cliAll, ".toInt()");
    expectNotContains(cliAll, ".toFloat()");
}

TEST(ProductionContracts, AircraftTypeLoggingContractsAreExplicit)
{
    const std::string state = readRepoFile("src/state/StateManagement.cpp");
    const std::string commands = readRepoFile("src/utils/CommandSystem.cpp");

    expectContains(state, "#if AIRCRAFT_TYPE == AIRCRAFT_TYPE_GLIDER");
    expectContains(state, "flashTelemetry.startLogging()");
    expectContains(state, "flashTelemetry.stopLogging()");

    expectContains(commands, "#if AIRCRAFT_TYPE == AIRCRAFT_TYPE_POWERED");
    expectContains(commands, "!controller->isThrottleCut()");
    expectContains(commands, "Throttle cut active");
    expectContains(commands, "case CMD_SET_THROTTLE_CUT");
    expectContains(commands, "!cmd.x_value && controller->isArmed()");
}

TEST(ProductionContracts, ImuSnapshotAndBarometerStayRealtimeSafe)
{
    const std::string imuH = readRepoFile("src/orientation/ArduFliteIMU.h");
    const std::string imuCpp = readRepoFile("src/orientation/ArduFliteIMU.cpp");

    // Lock-free seqlock writer/reader retained, including the retry-limit fallback.
    expectContains(imuH, "snapshotCurrent");
    expectContains(imuH, "snapshotVersion");
    expectContains(imuCpp, "snapshotVersion.store(version + 1");
    expectContains(imuCpp, "snapshotVersion.store(version + 2");
    expectContains(imuCpp, "if (before == after) return snap;");
    expectContains(imuCpp, "getLastCompleteSnapshot()");
    expectContains(imuCpp, "snapshotRetryLimitHits");

    // The separate Baro Task is GONE — the IMU task is the sole I2C-bus owner.
    // Reintroducing it reintroduces the priority-inversion "sensor mutex busy" skips.
    expectNotContains(imuH, "static void baroTask");
    expectNotContains(imuCpp, "xTaskCreate(baroTask");
    expectNotContains(imuCpp, "void ArduFliteIMU::baroTask");

    // Baro is read INLINE in update(), decimated, with the climb rate gated on baro
    // ticks, and the coherent snapshot still published from the IMU task.
    expectContains(imuH, "BARO_DECIMATION_FACTOR");
    expectContains(imuH, "_baroTickCounter");

    const std::string updateBody = sliceBetween(
        imuCpp,
        "void ArduFliteIMU::update(float dt)",
        "void ArduFliteIMU::applyOrientation()");
    expectContains(updateBody, "_baroTickCounter >= BARO_DECIMATION_FACTOR");
    expectContains(updateBody, "readBaroAltitude()");
    expectContains(updateBody, "_baroFilterInitialized");
    expectContains(updateBody, "climbRate");
    expectContains(updateBody, "publishSnapshot();");

    // update() must guard the mutex acquire — never drive the I2C bus / publish the
    // snapshot without holding imuMutex.
    expectContains(updateBody, "if (!lock.acquired())");
}
