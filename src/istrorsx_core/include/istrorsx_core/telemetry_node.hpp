#pragma once

#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"

// ROS port of legacy script/logp.cpp -- a standalone, non-ROS log-tailing
// utility (compiled/run completely separately from istro_rt2025 itself, no
// Config/Threads/DataSet round trip, unlike every other node in this
// project -- legacy logp.cpp never shared any code with the main
// application) that polls the shared application log file, extracts the
// latest value of ~20 hand-picked LOGM_* patterns via plain substring
// search, and periodically (re)writes the accumulated results as txt/html/
// json for a simple web dashboard (html/logp_request.js, not ported yet)
// to poll.
//
// User framing for the eventual redesign (see 01_architecture.md): "logp
// bude stručne telemetry_node... backend pre vizualizér, ktorý sprístupňuje
// historické a aj online data" -- but THIS round is explicitly scoped to a
// faithful 1:1 port of the parsing/output mechanism only ("nech funguje
// úplne 1:1 parsuje logy a vytvára txt"), no ROS topics/services yet. The
// online/historical-data-serving redesign is a deliberate follow-up, not
// started here.
//
// No istro_main_<subsystem>() scaffold, unlike every other node -- legacy
// logp.cpp never used Config/Threads/DataSet at all, so there is no Config
// to parse or DataSet to wire up. main() still inlines the one piece every
// istro_main_<subsystem>() otherwise provides that this node genuinely
// needs: LOG_CONFIG_LOAD("conf/log4cxx.xml") (+LOG_THREAD_NAME/LOG_INFO
// startup banner) -- without it log4cxx has no appender configured for any
// LOGM_*/LOG_* call at all (found via testing: "No appender could be found
// for logger (telemetry)"), silently swallowing every log line instead of
// writing them.
//
// Reads LOG_FNAME (telemetry_node.cpp, "logout/istro_<EVENT_TAG>.log") -- the
// same shared log4cxx output file every other node's own LOGM_* calls
// already write into (confirmed via direct observation this session: e.g.
// istroSave::/istroVision:: lines from different processes land in the same
// rotated file), so legacy's single-process "one big combined log" property
// still holds here despite each subsystem now being its own process.
class TelemetryNode : public rclcpp::Node {
public:
    TelemetryNode();
    ~TelemetryNode();

    // Public so telemetry_node.cpp's own file-scope kFindPattern[] (mirrors
    // legacy's own file-scope findPattern[]) can use it as an array bound.
    static const int FIND_PATTERN_COUNT = 20;

private:
    // ---- Port of logp.cpp's own free functions (istro_rt2025 script/logp.cpp) ----
    void tick(void);              // was main()'s own while(true) loop body (minus the sleep -- the timer paces this instead)
    void resetState(void);        // was init()
    void processLine(const char *p, size_t n);  // was process_line() -- unchanged algorithm, see .cpp
    void writeResult(void);       // was writeResult()

    // Not part of legacy logp.cpp -- telemetry_node is now the single owner
    // of log rotation (conf/log4cxx.xml's appender no longer rotates itself,
    // see its comment). "copytruncate": copies the live log aside to a
    // timestamped name, then truncates the live file in place (same inode,
    // no rename) so every writer's already-open, append-mode fd keeps working
    // without needing to reopen anything. See .cpp for the race this replaces.
    void rotateLog(void);

    static const int LINE_SIZE = 1024;
    static const size_t BUFFER_SIZE = 1024 * 1024;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string log_fname_;     // fixed -- this project's own logout/ convention, not user-configurable (see .cpp)
    std::string output_dir_;    // ROS parameter "output_dir", default "ramdisk/" -- user explicitly wants this configurable, unlike legacy's own hardcoded /var/www/html/ramdisk/ (a Jetson-specific web-server path that doesn't exist in this dev setup)

    // ---- logp.cpp's own file-scope state (buffer[]/buffer_cnt/off/
    // read_empty/findResult[]), member-ized but otherwise unchanged --
    // see .cpp for why read_empty_ never resets on a successful read
    // (deliberate self-healing, not a bug -- fread() can't tell "genuine
    // error" apart from "at EOF, nothing new yet"). ----
    char buffer_[BUFFER_SIZE + 1];
    size_t buffer_cnt_ = 0;
    long off_ = 0;
    int read_empty_ = 0;
    char findResult_[FIND_PATTERN_COUNT][LINE_SIZE + 1];

    FILE *fd_ = nullptr;
    int sleept_ = 0;   // ms since the last GZIP_PERIOD trigger, see .cpp
    int rotate_sleept_ = 0;   // ms since the last ROTATE_PERIOD trigger (rotateLog()), see .cpp
};
