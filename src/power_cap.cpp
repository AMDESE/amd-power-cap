#include "power_cap.hpp"

#include <boost/asio.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/spawn.hpp>
#include <gpiod.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/property.hpp>

#include <filesystem>

#include "iomanip"

extern "C"
{
#include "apml.h"
#include "esmi_mailbox.h"
#include "esmi_rmi.h"

#include <unistd.h>
}

#define COMMAND_NUM_OF_CPU ("/sbin/fw_printenv -n num_of_cpu")
#define COMMAND_LEN 3
#define SMU_INIT_WAIT 180
#define MAX_RETRY 10
#define CPU_MAX_PWR_LIMIT (1000) // 1000 watts, max perf

const std::string PwrOkName = "MON_POST_COMPLETE";
constexpr auto POWER_SERVICE = "xyz.openbmc_project.Settings";
constexpr auto POWER_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";
constexpr auto POWER_CAP_STR = "PowerCap";
constexpr auto POWER_CAP_ENABLE_STR = "PowerCapEnable";
constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

// HostMode (HPAR) settings object
constexpr auto HOSTMODE_PATH = "/xyz/openbmc_project/control/HostMode";
constexpr auto HOSTMODE_INTERFACE = "xyz.openbmc_project.Control.HostMode";
constexpr auto HOSTMODE_CURRENT_STR = "CurrentMode";

// APML soc_die_num values (see apml_library: bits[3:0]=socket, bits[7:4]=die)
constexpr uint8_t P0_SOC_DIE_NUM = 0; // Socket 0, Die 0
constexpr uint8_t P1_SOC_DIE_NUM = 1; // Socket 1, Die 0

PowerCapDataHolder* PowerCapDataHolder::instance = 0;

// Set power limit to CPU using OOB library
uint32_t PowerCap::set_oob_pwr_limit(uint8_t bus, uint32_t req_pwr_limit)
{
    oob_status_t ret;
    uint32_t current_pwr_limit;

    ret = read_socket_power_limit(bus, &current_pwr_limit);
    if ((ret == OOB_SUCCESS) && (current_pwr_limit != 0))
    {
        sd_journal_print(LOG_DEBUG, "Initial Power Cap Value %d \n",
                         current_pwr_limit);
    }
    else
    {
        sd_journal_print(LOG_INFO, "unable to read power limit \n");
        return -1;
    }

    /* CPU is already running at requested limit
     * OOB deals in milliwatts only */
    if ((current_pwr_limit / 1000) == req_pwr_limit)
    {
        sd_journal_print(LOG_DEBUG,
                         "CPU already operating at requested power limit \n");
        return req_pwr_limit;
    }

    // Set user supplied limit to CPU (in milliwatts)
    ret = write_socket_power_limit(bus, (req_pwr_limit * 1000));
    if (ret != OOB_SUCCESS)
    {
        sd_journal_print(LOG_ERR, "Setting power cap value failed \n");
        return -1;
    }
    else
    {
        sd_journal_print(LOG_INFO, "Power Limit Set Successfully\n");
    }

    // Readback and confirm the max limit accepted by CPU
    // if CPU doesnt support user limit, it returns its default power limit
    ret = read_socket_power_limit(bus, &current_pwr_limit);
    if ((ret == OOB_SUCCESS) && (current_pwr_limit != 0))
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Updated Power Cap Value ",
            phosphor::logging::entry("Updated Power Cap Value %d",
                                     current_pwr_limit));
        return (current_pwr_limit / 1000);
    }
    else
    {
        sd_journal_print(LOG_ERR, "Readback power cap value failed \n");
        return -1;
    }

    return -1;
}

// Determine which APML socket(s) this daemon instance is responsible for.
//   host0 (2P)   -> socket 0, and socket 1 too when a 2nd CPU is present
//   host1 (2x1P) -> socket 0 (P0)
//   host2 (2x1P) -> socket 1 (P1)
void PowerCap::build_socket_list()
{
    socketList.clear();

    switch (hostInstance)
    {
        case 1: // 2x1P, host1 == P0
            socketList.push_back(P0_SOC_DIE_NUM);
            break;
        case 2: // 2x1P, host2 == P1
            socketList.push_back(P1_SOC_DIE_NUM);
            break;
        case 0: // 2P, single OS spanning all present sockets
        default:
            socketList.push_back(P0_SOC_DIE_NUM);
            if (num_of_proc == 2)
            {
                socketList.push_back(P1_SOC_DIE_NUM);
            }
            break;
    }
}

// Read current HPAR mode from HostMode settings.
// Returns HPAR_MODE_2P (0) on any error / when HostMode is unavailable, so
// single-socket and legacy 2P platforms keep their default behavior.
int PowerCap::get_hpar_mode()
{
    try
    {
        std::string settingManager =
            getService(bus, HOSTMODE_PATH, HOSTMODE_INTERFACE);
        if (settingManager.empty())
            return HPAR_MODE_2P;

        auto method =
            bus.new_method_call(settingManager.c_str(), HOSTMODE_PATH,
                                "org.freedesktop.DBus.Properties", "Get");
        method.append(HOSTMODE_INTERFACE, HOSTMODE_CURRENT_STR);

        // CurrentMode representation may vary; accept the common encodings.
        std::variant<std::string, bool, uint8_t, uint16_t, uint32_t, uint64_t,
                     int16_t, int32_t, int64_t>
            value{};
        auto reply = bus.call(method);
        reply.read(value);

        return std::visit(
            [](auto&& val) -> int {
                using T = std::decay_t<decltype(val)>;
                if constexpr (std::is_same_v<T, std::string>)
                {
                    try
                    {
                        return std::stoi(val);
                    }
                    catch (...)
                    {
                        return HPAR_MODE_2P;
                    }
                }
                else if constexpr (std::is_same_v<T, bool>)
                {
                    return val ? HPAR_MODE_2X1P : HPAR_MODE_2P;
                }
                else
                {
                    return static_cast<int>(val);
                }
            },
            value);
    }
    catch (const std::exception& ex)
    {
        sd_journal_print(LOG_ERR, "Unable to read HostMode, assuming 2P \n");
    }
    return HPAR_MODE_2P;
}

// host0 instance services 2P mode only; host1/host2 service 2x1P mode only.
// This prevents two instances from driving the same socket when all three
// power_cap settings objects exist simultaneously.
bool PowerCap::is_instance_applicable()
{
    int mode = get_hpar_mode();
    if (hostInstance == 0)
    {
        return (mode == HPAR_MODE_2P);
    }
    return (mode == HPAR_MODE_2X1P);
}

// read stored settings, user requested limit and apply power cap
bool PowerCap::do_power_capping()
{
    int ret = -1;
    bool any_success = false;
    bool writeback_done = false;

    // Only act if this instance matches the running HPAR mode.
    if (!is_instance_applicable())
    {
        sd_journal_print(LOG_DEBUG,
                         "host%d not applicable for current HPAR mode \n",
                         hostInstance);
        return true;
    }

    /* Do nothing, if new limit is same as old */
    if (AppliedPowerCapData == static_cast<int>(userPCapLimit))
        return true;

    // Apply the requested limit to every socket owned by this instance.
    // In 2x1P each instance owns exactly one socket, giving each independent
    // CPU its own power cap. In 2P (host0) the same value is applied to all
    // present sockets.
    for (uint8_t soc_die_num : socketList)
    {
        ret = PowerCap::set_oob_pwr_limit(soc_die_num, userPCapLimit);
        if (ret > 0)
        {
            any_success = true;
            // update d-bus property if CPU applied a different limit
            // Assume we have a 240W CPU part, but user requests 320W
            // CPU will report 240W since it is the max.
            if ((ret != static_cast<int>(userPCapLimit)) && !writeback_done)
            {
                PowerCap::set_power_cap_limit(ret);
                writeback_done = true;
            }
        }
    }

    if (any_success)
    {
        sd_journal_print(LOG_DEBUG, "AppliedPowerCapData %d\n",
                         AppliedPowerCapData);
        AppliedPowerCapData = userPCapLimit;
    }

    return any_success;
}

void PowerCap::get_num_of_proc()
{
    // Default to a single processor; only override on a clean read. This must
    // never throw: platform.json can be missing, empty or partially written
    // early in boot, and an uncaught exception here would abort the daemon.
    num_of_proc = 1;

    std::ifstream file("/var/lib/platform-config/platform.json");
    if (!file.is_open())
    {
        sd_journal_print(LOG_INFO,
                         "platform.json not present, assuming 1 CPU \n");
        return;
    }

    // parse() with allow_exceptions=false returns a discarded value instead
    // of throwing when the input is empty or malformed.
    nlohmann::json jsonData = nlohmann::json::parse(file, nullptr, false);
    if (jsonData.is_discarded())
    {
        sd_journal_print(LOG_INFO,
                         "platform.json not ready, assuming 1 CPU \n");
        return;
    }

    if (jsonData.contains("CpuCount"))
    {
        num_of_proc = jsonData["CpuCount"].get<int>();
    }
    else
    {
        sd_journal_print(LOG_ERR,
                         "CpuCount missing in platform.json, assuming 1 \n");
    }
}

int PowerCap::getGPIOValue(const std::string& name)
{
    int value;
    gpiod::line gpioLine;

    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        sd_journal_print(LOG_ERR, "Can't find line: %s \n", name.c_str());
        return -1;
    }
    try
    {
        gpioLine.request(
            {__FUNCTION__, gpiod::line_request::DIRECTION_INPUT, 0});
    }
    catch (std::system_error& exc)
    {
        sd_journal_print(LOG_ERR, "Error setting gpio as Input: %s \n",
                         name.c_str());
        return -1;
    }

    try
    {
        value = gpioLine.get_value();
    }
    catch (std::system_error& exc)
    {
        sd_journal_print(LOG_ERR, "Error getting gpio value for: %s \n",
                         name.c_str());
        return -1;
    }

    return value;
}

int system_check(char* cmd)
{
    int rc = system(cmd);
    if (rc < 0)
        sd_journal_print(LOG_ERR, "Failed to run system cmd: %s \n", cmd);
    return rc;
}

// CPU loses the power limit applied after reboot
// re-apply previous value from BMC NV
void PowerCap::onHostPwrChange()
{
    uint32_t retry = 0;
    bool status = false;

    status = get_power_cap_enabled_setting();

    if (status && PowerCapEnableData == true)
    {
        get_power_cap_limit();

        // loop until SMU firmware initalizes
        while ((do_power_capping() == false) && (retry < MAX_RETRY))
        {
            sleep(30);
            sd_journal_print(LOG_INFO, "SMU not initialized, retrying...\n");
            retry++;
        }
    }
    else
    {
        sd_journal_print(LOG_ERR, "Power cap not enabled \n");
    }
}
void PowerCap::init_power_capping()
{
    uint32_t retry = 0;
    bool status = false;

    status = get_power_cap_enabled_setting();

    while ((status == false) && (retry < MAX_RETRY))
    {
        sleep(10); // retry in 10s interval till phosphor-settings service loads
        status = get_power_cap_enabled_setting();
        retry++;
    }

    if (status && (PowerCapEnableData == true))
    {
        PowerCap::get_power_cap_limit();

        // if host is off when BMC booted, this will do nothing
        // power cap settings will be applied when host power state changes
        PowerCap::do_power_capping();
    }
    else if (retry >= MAX_RETRY)
    {
        sd_journal_print(LOG_ERR, "Power cap settings not found \n");
    }
}

void PowerCap::get_power_cap_limit()
{
    std::string settingManager =
        getService(bus, powerCapPath.c_str(), POWER_INTERFACE);

    AppliedPowerCapData =
        getProperty<uint32_t>(bus, settingManager.c_str(), powerCapPath.c_str(),
                              POWER_INTERFACE, POWER_CAP_STR);
}

bool PowerCap::get_power_cap_enabled_setting()
{
    try
    {
        std::string settingManager =
            getService(bus, powerCapPath.c_str(), POWER_INTERFACE);
        if (settingManager.empty())
            return false;

        PowerCapEnableData = getProperty<bool>(
            bus, settingManager.c_str(), powerCapPath.c_str(), POWER_INTERFACE,
            POWER_CAP_ENABLE_STR);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        sd_journal_print(LOG_ERR, "sdbus error \n");
    }
    return true;
}

template <typename T>
T PowerCap::getProperty(sdbusplus::bus::bus& bus, const char* service,
                        const char* path, const char* interface,
                        const char* propertyName)
{
    auto method = bus.new_method_call(service, path,
                                      "org.freedesktop.DBus.Properties", "Get");
    method.append(interface, propertyName);
    std::variant<T> value{};
    try
    {
        auto reply = bus.call(method);
        reply.read(value);
        return std::get<T>(value);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        sd_journal_print(LOG_ERR, "GetProperty call failed \n");
        return T{};
    }
}

std::string PowerCap::getService(sdbusplus::bus::bus& bus, const char* path,
                                 const char* interface)
{
    auto mapper = bus.new_method_call(MAPPER_BUSNAME, MAPPER_PATH,
                                      MAPPER_INTERFACE, "GetObject");

    mapper.append(path, std::vector<std::string>({interface}));
    try
    {
        auto mapperResponseMsg = bus.call(mapper);

        std::vector<std::pair<std::string, std::vector<std::string>>>
            mapperResponse;
        mapperResponseMsg.read(mapperResponse);
        if (mapperResponse.empty())
        {
            sd_journal_print(LOG_ERR, "Error reading mapper response \n");
        }
        if (mapperResponse.size() < 1)
        {
            return "";
        }
        return mapperResponse[0].first;
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        sd_journal_print(LOG_ERR, "Mapper call failed \n");
    }
    return "";
}

void PowerCap::set_power_cap_limit(uint32_t value)
{
    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    boost::system::error_code ec;
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->async_method_call(
        [this](boost::system::error_code ec) {
            if (ec)
            {
                sd_journal_print(
                    LOG_ERR,
                    "Failed to set power cap value in dbus interface \n");
            }
        },
        POWER_SERVICE, powerCapPath.c_str(), "org.freedesktop.DBus.Properties",
        "Set", POWER_INTERFACE, POWER_CAP_STR, std::variant<uint32_t>(value));
    AppliedPowerCapData = value;
}
