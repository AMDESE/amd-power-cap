#include "power_cap.hpp"

#include "iomanip"
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/error.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/property.hpp>
#include <gpiod.hpp>
#include <filesystem>
#include <linux/types.h>
#include <linux/ioctl.h>

extern "C" {
#include <unistd.h>
#include "linux/i2c-dev.h"
#include "i2c/smbus.h"
#include "apml.h"
#include "esmi_mailbox.h"
#include "esmi_rmi.h"
}

#define COMMAND_NUM_OF_CPU  ("/sbin/fw_printenv -n num_of_cpu")
#define COMMAND_LEN         3
#define SMU_INIT_WAIT       180
#define MAX_RETRY           10
#define CPU_MAX_PWR_LIMIT   (1000) //1000 watts, max perf

// Definition for I3C APML
#define MAX_APML_BUS     2
#define I3C_BUS_APML0    4
#define I3C_BUS_APML1    5
#define CMD_BUFF_LEN     256

// IOCTL command
#define I3C_DEV_IOC_MAGIC 0x07

#define APML_INIT_DONE_FILE	"/tmp/apml_init_complete"

/**
 * struct i3c_ioc_priv_xfer - I3C SDR ioctl private transfer
 * @data: Holds pointer to userspace buffer with transmit data.
 * @len: Length of data buffer buffers, in bytes.
 * @rnw: encodes the transfer direction. true for a read, false for a write
 */
struct i3c_ioc_priv_xfer {
        __u64 data;
        __u16 len;
        __u8 rnw;
        __u8 pad[5];
};

const     int  I3C_BUS[2] = {I3C_BUS_APML0 , I3C_BUS_APML1};

const std::string PwrOkName = "MON_POST_COMPLETE";
constexpr auto POWER_SERVICE = "xyz.openbmc_project.Settings";
std::string POWER_PATH ="/xyz/openbmc_project/control/host0/power_cap";
constexpr auto POWER_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";
constexpr auto POWER_CAP_STR = "PowerCap";
constexpr auto POWER_CAP_ENABLE_STR = "PowerCapEnable";
constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

PowerCapDataHolder* PowerCapDataHolder::instance = 0;

uint8_t p0_info = 0;
uint8_t p1_info = 1;

// Set power limit to CPU using OOB library
uint32_t PowerCap::set_oob_pwr_limit (uint8_t bus, uint32_t req_pwr_limit)
{
    oob_status_t ret;
    uint32_t current_pwr_limit;

    ret = read_socket_power_limit(bus, &current_pwr_limit);
    if((ret == OOB_SUCCESS) && (current_pwr_limit != 0))
    {
        sd_journal_print(LOG_DEBUG, "Initial Power Cap Value %d \n",current_pwr_limit);
    }
    else
    {
        sd_journal_print(LOG_INFO, "unable to read power limit \n");
        return -1;
    }

    /* CPU is already running at requested limit
     * OOB deals in milliwatts only */
    if ((current_pwr_limit/1000) == req_pwr_limit)
    {
        sd_journal_print(LOG_DEBUG, "CPU already operating at requested power limit \n");
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
        sd_journal_print(LOG_INFO,"Power Limit Set Successfully\n");
    }

    // Readback and confirm the max limit accepted by CPU
    // if CPU doesnt support user limit, it returns its default power limit
    ret = read_socket_power_limit(bus, &current_pwr_limit);
    if((ret == OOB_SUCCESS) && (current_pwr_limit != 0))
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Updated Power Cap Value ",phosphor::logging::entry(
            "Updated Power Cap Value %d",current_pwr_limit));
        return (current_pwr_limit/1000);
    }
    else
    {
        sd_journal_print(LOG_ERR, "Readback power cap value failed \n");
        return -1;
    }

    return -1;
}

// read stored settings, user requested limit and apply power cap
bool PowerCap::do_power_capping() {

    int ret = -1;
    bool set_powercap = false;

    /* Do nothing, if new limit is same as old */
    if (AppliedPowerCapData == userPCapLimit)
        return true;

    //P0 Power Cap Value Update
    ret = PowerCap::set_oob_pwr_limit(p0_info, userPCapLimit);
    // update d-bus property if CPU applied a different limit
    // Assume we have a 240W CPU part, but user requests 320W
    // CPU will report 240W since it is the max.
    if ((ret > 0) && (ret != userPCapLimit)) {
        // socket P0 set was successful
        // We assume both sockets have same OPN
        PowerCap::set_power_cap_limit(ret);
        set_powercap = true;
    }

    // P1 Power Cap Value Update
    if (num_of_proc == 2)
    {
        ret = PowerCap::set_oob_pwr_limit(p1_info, userPCapLimit);
        // TBD: check if 2P config supports different OPNs
        if (set_powercap == false) {
            if ((ret > 0) && (ret != userPCapLimit)) {
                PowerCap::set_power_cap_limit(ret);
                set_powercap = true;
            }
	}
    }

    return set_powercap;
}

bool PowerCap::get_num_of_proc()
{
    FILE *pf;
    char data[COMMAND_LEN];
    std::stringstream ss;

    num_of_proc = 1;
    // Setup pipe for reading and execute to get u-boot environment
    // variable board_id.
    pf = popen(COMMAND_NUM_OF_CPU,"r");

    if(pf > 0)
    {   // no error
        if (fgets(data, COMMAND_LEN , pf) != NULL)
        {
            ss << std::hex << (std::string)data;
            ss >> num_of_proc;
        }
            pclose(pf);
            return true;
    }
    else
    {
        sd_journal_print(LOG_ERR, "Failed to open command stream \n");
    }

    return false;
}

int  PowerCap::getGPIOValue(const std::string& name)
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
        gpioLine.request({__FUNCTION__, gpiod::line_request::DIRECTION_INPUT});
    }
    catch (std::system_error& exc)
    {
        sd_journal_print(LOG_ERR, "Error setting gpio as Input: %s \n", name.c_str());
        return -1;
    }

    try
    {
        value = gpioLine.get_value();
    }
    catch (std::system_error& exc)
    {
        sd_journal_print(LOG_ERR, "Error getting gpio value for: %s \n", name.c_str());
        return -1;
    }

    return value;
}

int system_check(char *cmd)
{
    int rc = system(cmd);
    if (rc < 0)
        sd_journal_print(LOG_ERR, "Failed to run system cmd: %s \n", cmd);
    return rc;
}

void PowerCap::unbind_APML_drivers()
{

    apml_unbind();
}

void PowerCap::bind_APML_drivers()
{
    int retry = 0;
    int bind_rc = 0;
    bool enableAPMLMux = false;

    while (retry < MAX_RETRY)
    {

        sleep(10);
        if (getGPIOValue(PwrOkName) > 0)
        {
            sd_journal_print(LOG_INFO, "POST Complete reached - Enable APML Mux \n");
            enableAPMLMux = true;
            break;
        }
        retry++;
    }
    if (enableAPMLMux == true)
    {
        if (apml_bind() >= 0)
        {
            // Touch a file to indicate APML slaves are configured
            std::ofstream initdone (APML_INIT_DONE_FILE);
            initdone.close();
        }
    }

    sd_journal_print(LOG_INFO, "APML MUX setting sucessful for %d CPU \n", num_of_proc);
}

// CPU loses the power limit applied after reboot
// re-apply previous value from BMC NV
void PowerCap::onHostPwrChange()
{
    uint32_t retry = 0;
    bool status = false;

    status = get_power_cap_enabled_setting();

    if(status && PowerCapEnableData == true)
    {
        get_power_cap_limit();

        // loop until SMU firmware initalizes
        while((do_power_capping() == false) && (retry < MAX_RETRY))
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

    while((status == false) && (retry < MAX_RETRY))
    {
        sleep(10); //retry in 10s interval till phosphor-settings service loads
        status = get_power_cap_enabled_setting();
        retry++;
    }

    if(status && (PowerCapEnableData == true))
    {
        PowerCap::get_power_cap_limit();

        // if host is off when BMC booted, this will do nothing
        // power cap settings will be applied when host power state changes
        PowerCap::do_power_capping();
    }
    else if(retry >= MAX_RETRY)
    {
        sd_journal_print(LOG_ERR, "Power cap settings not found \n");
    }
}

void PowerCap::get_power_cap_limit()
{
    std::string settingManager = getService(bus, POWER_PATH.c_str() , POWER_INTERFACE);

    AppliedPowerCapData = getProperty<uint32_t>(bus, settingManager.c_str(),
                                           POWER_PATH.c_str(),
                                           POWER_INTERFACE, POWER_CAP_STR) ;
}

bool PowerCap::get_power_cap_enabled_setting()
{
    try
    {
        std::string settingManager = getService(bus, POWER_PATH.c_str() , POWER_INTERFACE);
        if (settingManager.empty())
            return false;

        PowerCapEnableData = getProperty<bool>(bus, settingManager.c_str(), POWER_PATH.c_str(),
                                           POWER_INTERFACE, POWER_CAP_ENABLE_STR) ;
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        sd_journal_print(LOG_ERR, "sdbus error \n");
    }
    return true;
}

template <typename T>
T PowerCap::getProperty(sdbusplus::bus::bus& bus, const char* service, const char* path,
              const char* interface, const char* propertyName)
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
                sd_journal_print(LOG_ERR, "Failed to set power cap value in dbus interface \n");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/control/host0/power_cap",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Control.Power.Cap", "PowerCap",
        std::variant<uint32_t>(value));
    AppliedPowerCapData = value;
}
