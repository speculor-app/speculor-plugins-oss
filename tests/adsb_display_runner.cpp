// adsb_display without a display.
//
// The rendering cannot be asserted headlessly, but the part that loses user
// data can: waypoints are a list parameter, and a list parameter is what a
// .speculor file stores and restores. If set_list_rows and get_list_rows
// disagree — a dropped column, a truncated string, a row silently discarded on
// overflow — a user's waypoints come back wrong after a reload, and nothing
// else in the system would notice.
//
// The rest is robustness: the plugin is interactive and non-blocking on every
// input, so it must tolerate being processed with nothing connected and being
// handed input events at times the GUI can genuinely produce them.

// before the harness, whose windows.h would otherwise bring in winsock.h first
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
constexpr socket_t kBadSocket = INVALID_SOCKET;
inline void close_socket(socket_t s) { ::closesocket(s); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
constexpr socket_t kBadSocket = -1;
inline void close_socket(socket_t s) { ::close(s); }
#endif

#include <testing/conformance.h>
#include <testing/fake_host.h>
#include <testing/plugin_under_test.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool cond, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), cond ? "ok" : "FAIL");
    if (!cond) ++failures;
}

// Never accepts on its own: the kernel completes the handshake from the backlog,
// so a request goes out and no reply ever comes.
class SilentServer {
public:
    bool start() {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
        sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == kBadSocket) return false;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return false;
        if (::listen(sock_, 8) != 0) return false;
        socklen_t len = sizeof(addr);
        if (::getsockname(sock_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) return false;
        port_ = ntohs(addr.sin_port);
        return true;
    }
    ~SilentServer() { if (sock_ != kBadSocket) close_socket(sock_); }

    // libcurl takes its proxy from the environment, so this catches every https:// request
    void proxy_https() const {
        const std::string proxy = "http://127.0.0.1:" + std::to_string(port_);
#ifdef _WIN32
        // libcurl reads the process environment block, not the CRT's copy
        SetEnvironmentVariableA("https_proxy", proxy.c_str());
        SetEnvironmentVariableA("no_proxy", nullptr);
#else
        setenv("https_proxy", proxy.c_str(), 1);
        unsetenv("no_proxy");
        unsetenv("NO_PROXY");
#endif
    }

    // the connection stays in the backlog until request_ended_within() takes it
    bool connected_within(int ms) const { return readable(sock_, ms); }

    // ended: the client closed the connection, as libcurl does on an aborted request
    bool request_ended_within(int ms) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        if (!connected_within(ms)) return false;
        const socket_t c = ::accept(sock_, nullptr, nullptr);
        if (c == kBadSocket) return false;
        bool ended = false;
        constexpr int kBufSize = 512;
        char buf[kBufSize];
        while (!ended) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (left <= 0 || !readable(c, static_cast<int>(left))) break;
            // the request arrives first; then the end of the stream, or a reset
            ended = ::recv(c, buf, kBufSize, 0) <= 0;
        }
        close_socket(c);
        return ended;
    }

private:
    static bool readable(socket_t s, int ms) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(s, &fds);
        timeval tv;
        tv.tv_sec = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;
        return ::select(static_cast<int>(s + 1), &fds, nullptr, nullptr, &tv) == 1;
    }

    socket_t sock_ = kBadSocket;
    int port_ = 0;
};

bool set_enum(spc::testing::PluginUnderTest& p, const char* name, int32_t v) {
    SpcParameterDesc d{};
    if (p.get_parameter(name, &d) != 0 || d.type != SPC_PARAM_ENUM) return false;
    d.enum_val.value = v;
    return p.set_parameter(name, &d) == 0;
}

bool set_string(spc::testing::PluginUnderTest& p, const char* name, const char* v) {
    SpcParameterDesc d{};
    if (p.get_parameter(name, &d) != 0 || d.type != SPC_PARAM_STRING) return false;
    std::snprintf(d.string_val.value, sizeof(d.string_val.value), "%s", v);
    return p.set_parameter(name, &d) == 0;
}

bool approx_eq(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

// Column order matches the descriptor: name, type(enum), lat, lon, alt.
SpcListRow make_waypoint(const char* name, int32_t type, double lat, double lon, double alt) {
    SpcListRow r{};
    std::strncpy(r.cells[0].string_val, name, SPC_LIST_CELL_STRING_MAX - 1);
    r.cells[1].enum_val = type;
    r.cells[2].float64_val = lat;
    r.cells[3].float64_val = lon;
    r.cells[4].float64_val = alt;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path-to-adsb_display>\n", argv[0]);
        return 2;
    }

    spc::testing::PluginUnderTest plugin;
    if (!plugin.load(argv[1])) {
        std::fprintf(stderr, "FAIL: %s\n", plugin.error().c_str());
        return 1;
    }
    const auto* d = plugin.descriptor();
    std::printf("plugin: %s\n", d && d->id[0] ? d->id : "<no id>");

    SilentServer server;
    if (!server.start()) {
        std::fprintf(stderr, "FAIL: could not open the silent server\n");
        return 1;
    }
    server.proxy_https();

    const SpcPlugin* vt = plugin.vtable();
    spc::testing::FakeHost host;

    check(plugin.create(), "create_instance");
    plugin.set_host_services(host.services());

    // --- waypoints round-trip ----------------------------------------------
    if (plugin.params() && plugin.params()->has_list()) {
        std::vector<SpcListRow> in;
        in.push_back(make_waypoint("Heathrow", 1, 51.4700, -0.4543, 83.0));
        in.push_back(make_waypoint("Gatwick", 1, 51.1537, -0.1821, 202.0));
        // A name at the buffer limit: truncation bugs hide behind short strings.
        std::string long_name(SPC_LIST_CELL_STRING_MAX - 1, 'x');
        in.push_back(make_waypoint(long_name.c_str(), 4, -33.9425, 151.1783, 21.0));

        const int set_rc = plugin.params()->set_list("waypoints",
                                             in.data(), static_cast<int32_t>(in.size()));
        check(set_rc == 0, "set_list_rows accepts waypoints");

        SpcListRow out[16]{};
        int32_t count = -1;
        const int get_rc = plugin.params()->get_list("waypoints", out, 16, &count);
        check(get_rc == 0, "get_list_rows succeeds");
        check(count == static_cast<int32_t>(in.size()),
              "row count survives the round-trip");
        std::printf("    wrote %zu rows, read back %d\n", in.size(), count);

        if (get_rc == 0 && count == static_cast<int32_t>(in.size())) {
            bool names_ok = true, coords_ok = true, enums_ok = true;
            for (size_t i = 0; i < in.size(); ++i) {
                if (std::strcmp(out[i].cells[0].string_val, in[i].cells[0].string_val) != 0)
                    names_ok = false;
                if (!approx_eq(out[i].cells[2].float64_val, in[i].cells[2].float64_val) ||
                    !approx_eq(out[i].cells[3].float64_val, in[i].cells[3].float64_val) ||
                    !approx_eq(out[i].cells[4].float64_val, in[i].cells[4].float64_val))
                    coords_ok = false;
                if (out[i].cells[1].enum_val != in[i].cells[1].enum_val) enums_ok = false;
            }
            check(names_ok, "names survive, including one at the buffer limit");
            // Coordinates are float64 in the schema; a float32 round-trip would
            // move a waypoint by metres.
            check(coords_ok, "lat/lon/alt survive without precision loss");
            check(enums_ok, "waypoint type enum survives");
        }

        // A host asking for fewer rows than exist must not be handed more than
        // it allocated — the classic buffer overrun in this shape of API.
        SpcListRow one_row[1]{};
        int32_t small_count = -1;
        const int trunc_rc = plugin.params()->get_list("waypoints",
                                               one_row, 1, &small_count);
        check(small_count <= 1,
              "get_list_rows respects max_rows and never overfills");
        std::printf("    max_rows=1 request returned rc=%d count=%d\n", trunc_rc, small_count);

        // Clearing is what "delete all waypoints" does in the GUI.
        check(plugin.params()->set_list("waypoints", nullptr, 0) == 0,
              "set_list_rows accepts an empty list");
        int32_t cleared = -1;
        plugin.params()->get_list("waypoints", out, 16, &cleared);
        check(cleared == 0, "cleared list reads back empty");

        // An unknown list name must be refused, not silently treated as one of
        // the real ones.
        int32_t bogus_count = -1;
        check(plugin.params()->get_list("__no_such_list__",
                                out, 16, &bogus_count) != 0,
              "get_list_rows rejects an unknown list name");
    } else {
        std::printf("  (no list parameter — skipping waypoint checks)\n");
    }

    // --- the Custom Server field is editable exactly when the style is Custom
    {
        SpcParameterDesc style{};
        check(plugin.get_parameter("map_style", &style) == 0 && style.enum_val.count == 6
                  && std::strcmp(style.enum_val.labels[5], "Custom") == 0,
              "Custom is the last of six map styles");
        const auto disabled = [&] {
            SpcParameterDesc dsc{};
            return plugin.get_parameter("custom_server", &dsc) == 0
                && (dsc.flags & SPC_PARAM_FLAG_DISABLED) != 0;
        };
        check(set_enum(plugin, "map_style", 2) && disabled(),
              "custom_server is disabled for a built-in style");
        check(set_enum(plugin, "map_style", 5) && !disabled(),
              "and editable for Custom, the last style");
    }

    // --- process with nothing connected -------------------------------------
    // Every input is NON_BLOCKING, so the engine will call process() with no
    // data whenever nothing upstream has produced any.
    // a server no run has cached, so the first frame asks for tiles (Custom is style 5)
    check(set_enum(plugin, "map_style", 5), "map_style Custom");
    check(set_string(plugin, "custom_server", "tiles.invalid"), "custom_server");
    check(plugin.start() == 0, "start()");
    SpcData outputs[1]{};
    outputs[0].type = SPC_DATA_FRAME;
    const int p_rc = plugin.process(nullptr, 0, outputs, 1);
    check(p_rc == 0 || p_rc < 0, "process() with no inputs returns without faulting");
    std::printf("    process(no inputs) returned %d%s\n", p_rc,
                outputs[0].frame ? " (produced a frame)" : " (no frame)");
    if (outputs[0].frame) {
        check(outputs[0].frame->width > 0 && outputs[0].frame->height > 0,
              "produced map frame has non-zero dimensions");
    }

    // --- input events -------------------------------------------------------
    // The GUI can deliver an event before the first frame; a plugin that
    // assumes otherwise faults on a stray click. After a stop there is no
    // instance to hand one to.
    // A null event is deliberately not tested: unlike scan_devices, whose ABI
    // comment states host_services may be null, the ui_input slot documents no
    // such allowance, and the engine calls it as ui_input(inst, &event)
    // from a reference — so null is unreachable. Asserting on it would invent a
    // contract and fail a plugin that correctly assumes a valid pointer.
    if (vt->ui_input && plugin.instance()) {
        SpcUiInput ev{};
        vt->ui_input(plugin.instance(), &ev);
        check(true, "ui_input tolerates a zeroed event");
    }

    // --- a change of tile server while a download hangs ---------------------
    check(server.connected_within(5000), "a tile download reaches the server");
    check(set_string(plugin, "custom_server", "tiles2.invalid"), "the tile server changes");
    const auto switching = std::chrono::steady_clock::now();
    SpcData next[1]{};
    next[0].type = SPC_DATA_FRAME;
    plugin.process(nullptr, 0, next, 1);
    const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - switching).count();
    check(took < 1000, "the step that switches servers returns at once ("
                           + std::to_string(took) + " ms)");
    check(server.request_ended_within(2500), "and the old server's download ends");

    // --- stop while a tile download hangs -----------------------------------
    // covers PLG-4
    check(server.connected_within(5000), "the new server's download starts");
    plugin.interrupt();
    check(server.request_ended_within(2500), "interrupt ends the tile download");
    check(plugin.stop() == 0, "stop()");

    plugin.destroy();
    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
