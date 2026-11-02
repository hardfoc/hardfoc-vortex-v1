/**
 * @file main.cpp
 * @brief vortex_node_host: the Vortex CANopen device as a host process.
 *
 *   vortex_node_host --node-id N --port vcan0|pipe:<path>
 *                    [--serial S] [--bitrate KBPS] [--nvs FILE]
 *
 * --node-id 255 starts LSS-unconfigured. --nvs keeps the sim NVS (the LSS
 * record) in FILE so a restart behaves like a power cycle. The process prints
 * one "ready" line on stdout once the node runs, and stops on SIGINT/SIGTERM.
 * `--port pipe:<path>` joins the pipe bus at <path> and hosts the hub there
 * if nobody does yet (hf-core PipePort).
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "CoNode.hpp"
#include "CoSvc.hpp"
#include "PipePort.h"
#include "SimAxisPort.hpp"
#include "SimNvs.hpp"
#if defined(__linux__)
#include "LinuxSocketCanPort.h"
#endif

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop.store(true); }

int Usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s --node-id N --port vcan0|pipe:<path> [--serial S] [--bitrate KBPS] [--nvs FILE]\n",
               argv0);
  return 2;
}

bool ParseUint(const char* s, unsigned long max, unsigned long& out) {
  char* end = nullptr;
  const unsigned long v = std::strtoul(s, &end, 0);
  if (end == s || *end != '\0' || v > max) {
    return false;
  }
  out = v;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  vortex::node::CoNodeConfig cfg;
  std::string port_spec;
  std::string nvs_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    unsigned long v = 0;
    if (i + 1 >= argc) {
      return Usage(argv[0]);
    }
    const char* val = argv[++i];
    if (a == "--node-id" && ParseUint(val, 255, v) && (v == 255 || (v >= 1 && v <= 127))) {
      cfg.node_id = static_cast<uint8_t>(v);
    } else if (a == "--port") {
      port_spec = val;
    } else if (a == "--serial" && ParseUint(val, 0xFFFFFFFFUL, v)) {
      cfg.serial_number = static_cast<uint32_t>(v);
    } else if (a == "--bitrate" && ParseUint(val, 1000, v)) {
      cfg.bitrate_kbps = static_cast<uint16_t>(v);
    } else if (a == "--nvs") {
      nvs_path = val;
    } else {
      return Usage(argv[0]);
    }
  }
  if (port_spec.empty()) {
    return Usage(argv[0]);
  }
  if (!nvs_path.empty()) {
    const auto valid = vortex::sim::ValidateNvsPath(nvs_path);
    if (!valid) {
      std::fprintf(stderr,
                   "vortex_node_host: --nvs %s refused: needs a regular file under the working directory or "
                   "$TMPDIR, without '..'\n",
                   nvs_path.c_str());
      return 2;
    }
    nvs_path = *valid;
  }

  std::unique_ptr<hf::can::ICanPort> port;
  if (port_spec.rfind("pipe:", 0) == 0) {
    hf::can::PipePort::Options o;
    o.path = port_spec.substr(5);
    auto p = std::make_unique<hf::can::PipePort>(o);
    if (!p->Open()) {
      std::fprintf(stderr, "vortex_node_host: cannot join the pipe bus at %s\n", o.path.c_str());
      return 1;
    }
    port = std::move(p);
  } else {
#if defined(__linux__)
    hf::can::LinuxSocketCanPort::Options o;
    o.ifname = port_spec;
    auto p = std::make_unique<hf::can::LinuxSocketCanPort>(o);
    if (!p->Open()) {
      std::fprintf(stderr, "vortex_node_host: cannot open SocketCAN interface %s\n", port_spec.c_str());
      return 1;
    }
    port = std::move(p);
#else
    std::fprintf(stderr, "vortex_node_host: SocketCAN needs Linux; use --port pipe:<path>\n");
    return 1;
#endif
  }

  vortex::sim::SimNvs nvs(nvs_path);
  vortex::sim::SimLssStore lss(nvs);
  vortex::sim::SimAxisPort axis;

  vortex::node::CoNode node(cfg);
  node.SetLssStore(&lss);
  if (!node.AddPort(vortex::contracts::router::Port::HostSim, *port, true) || !node.Start()) {
    std::fprintf(stderr, "vortex_node_host: CANopen initialisation failed\n");
    return 1;
  }
  vortex::node::CoSvc svc(node);
  if (!svc.Start()) {
    std::fprintf(stderr, "vortex_node_host: cannot start co_svc\n");
    return 1;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  std::signal(SIGPIPE, SIG_IGN);
  std::printf("vortex_node_host: node %u on %s ready\n", static_cast<unsigned>(node.ActiveNodeId()),
              port_spec.c_str());
  std::fflush(stdout);

  while (!g_stop.load() && svc.Running()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    axis.Step(10000U);
  }
  svc.Stop();
  hf_co_port_stats_t st;
  hf_co_driver_get_stats(node.Driver(), static_cast<uint8_t>(vortex::contracts::router::Port::HostSim), &st);
  std::printf(
      "vortex_node_host: rx %u dispatched %u unhandled %u denied %u overruns %u | tx %u queue-full %u retried %u "
      "dropped %u denied %u | router drops %u\n",
      st.rx_frames, st.rx_dispatched, st.rx_unhandled, st.rx_denied, st.rx_ring_overruns, st.tx_frames,
      st.tx_queue_full, st.tx_retried, st.tx_dropped, st.tx_denied,
      node.Router().Drops(vortex::contracts::router::Port::HostSim));
  node.Stop();
  std::printf("vortex_node_host: stopped (%u service passes, longest %u us)\n", svc.Passes(), svc.MaxPassUs());
  return 0;
}
