#include <iostream>

#include "irsl/shm_controller.h"
#include "irsl/realtime_task.h"
#include "irsl/simple_yaml_parser.hpp"
#include "irsl/signal_handler.h"
#include "irsl/opt_parse.hpp"

#include "irsl_dynamixel_hardware_shm/DynamixelShmLib.h"

namespace icu = irsl_common_utils;
namespace isc = irsl_shm_controller;
namespace irt = irsl_realtime_task;

/// for debug
void status_print(
    const floatvec& cur_pos_flt,
    const floatvec& cur_vel_flt)
{
    size_t n = std::min(cur_pos_flt.size(), cur_vel_flt.size());

    for (size_t i = 0; i < n; i++)
    {
        std::cerr << i << " "
                  << cur_pos_flt[i] << " "
                  << cur_vel_flt[i] << std::endl;
    }
}

////
bool _running_ = true;
void _sighandle(int sig)
{
    _running_ = false;
}
///
int main(int argc, char **argv)
{
    setSignalHandler(SIGINT, _sighandle);

    ////
    std::string fname;
    bool verbose = false;
    icu::OptParse op("Dynamixel controller");
    op.add_option("--config", fname, "name of input file(.yaml)")->default_val("config.yaml");
    op.add_flag("-v,--verbose", verbose, "verbose message");
    op.opt_parse(argc, argv);

    ////
    irsl_dynamixel::DynamixelShm ds;
    ds.initialize(fname, op.getHash(), op.getShmKey());
    //
    ds.readDx();
    ds.initializeCommand();

    ////
    unsigned long interval_ns = (unsigned long)(ds.getPeriod() * 1000000000);
    std::cout << "start with period: " << ds.getPeriod() << std::endl;
    irt::RealtimeContext rt(interval_ns);
    int cntr = 0;

    ///
    rt.start();
    while (_running_) {
        ds.writeDx();
        ds.readDx();

        cntr++;

        ds.incrementFrame();
        if (cntr > 100) {
            if (verbose ) {
                std::cerr << "max: " << rt.getMaxInterval() << std::endl;
            }
            rt.reset();
            cntr = 0;
        }
        rt.waitNextFrame();
    }
    // polling
    ds.finalize();
    return 0;
}
