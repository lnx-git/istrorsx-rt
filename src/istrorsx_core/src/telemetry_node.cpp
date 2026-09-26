#include "istrorsx_core/telemetry_node.hpp"

#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>

#include "event_defs.h"
#include "logger.h"

LOG_DEFINE(loggerTelemetry, "telemetry");

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// logp.cpp's own file-scope constants -- LOG_FNAME/GZIP_CMD adapted from
// legacy's absolute Jetson paths (/home/istrobotics/projects/istro_rt2025/...)
// to this workspace's own logout/ convention, matching every other node's
// own use of workspace-relative out/logout paths (e.g. save_node.cpp's
// outputDir="out/"). The three legacy OUT_*_FNAME constants are gone --
// see TelemetryNode's own output_dir_ (ROS parameter, .hpp).
//
// EVENT_TAG (event_defs.h) instead of a literal tag here -- but
// conf/log4cxx.xml's own <param name="file"/> (the actual filename log4cxx
// itself writes to) is a separate XML file the C preprocessor can't reach,
// so it has its own copy of the tag that must be kept in sync by hand whenever
// EVENT_TAG changes, or LOG_FNAME/GZIP_CMD below will silently point at a
// file log4cxx never writes.
//
// GZIP_CMD gains a "-f" over legacy's bare "gzip": the rotated names
// rotateLog() produces repeat (they are "%H_%M", so once a day, and again
// after any restart within the same minute-of-day), and gzip asks
// "...already exists; do you wish to overwrite?" on stdin when its target
// .gz is already there. Under system() nothing ever answers, so the whole
// node hung. "-f" overwrites instead of asking. Deliberately NOT "-k":
// letting gzip delete each .log once compressed is what keeps logout/ from
// growing without bound, and it means each file is compressed exactly once
// rather than re-compressed on every GZIP_PERIOD tick. Consequence for
// grepping: rotated logs are .log.gz, so use zgrep for anything older than
// the live file (doc/ai/06_hw_testing_guide.md says so too).
//
// The "ls ... && " guard exists because dropping "-k" made the no-match case
// the common one: rotation happens once a minute but this runs every
// GZIP_PERIOD (15 s), so three runs out of four find every rotated log
// already compressed and removed. An unmatched glob is passed through to
// gzip literally, which then prints "No such file or directory" to the
// node's console every 15 seconds. The guard skips the call instead, and
// unlike a blanket "2>/dev/null" it still lets real gzip errors through.
// ---------------------------------------------------------------------------

static const char *LOG_FNAME = "logout/istro_" EVENT_TAG ".log";
static const char *GZIP_CMD  =
    "ls logout/istro_" EVENT_TAG ".*.log >/dev/null 2>&1 && "
    "gzip -f logout/istro_" EVENT_TAG ".*.log";
static const char *OUT_IMG_DNAME = "out/";

// Not legacy constants -- tick()'s own timer period (matches legacy's own
// msleep(250) exactly) and the gzip trigger threshold (matches legacy's own
// "sleept >= 15000").
static const int TELEMETRY_TICK_PERIOD = 250;   // (ms)
static const int GZIP_PERIOD           = 15000; // (ms)

// Not a legacy constant either -- telemetry_node's own log rotation period
// (rotateLog()), replacing conf/log4cxx.xml's removed per-process
// TimeBasedRollingPolicy. Matches its old "%d{HH_mm}" granularity (once a
// minute) so the archived filenames/cadence look the same from the outside.
static const int ROTATE_PERIOD = 60000; // (ms)

static const int FIND_FLAGS_CAMERA_IMG  = 1;
static const int FIND_FLAGS_VISION_IMG  = 2;
static const int FIND_FLAGS_LIDAR_IMG   = 3;
static const int FIND_FLAGS_WMGRID_IMG  = 4;
static const int FIND_FLAGS_NAVMAP_IMG  = 5;
static const int FIND_FLAGS_CDEPTH_IMG  = 6;
static const int FIND_FLAGS_CAMERA2_IMG = 7;
static const int FIND_FLAGS_CDEPTH2_IMG = 8;
static const int FIND_FLAGS_VISION2_IMG = 9;

struct FindPattern {
    const char *str1;
    const char *str2;
    int dd;      // distance in characters
    int flags;
};

// ---------------------------------------------------------------------------
// findPattern[] -- 1:1 port of logp.cpp's own table. 14 of the 20 entries
// matched real ported log output verbatim, confirmed against actual runtime
// LOGM_* output from this session's own testing (not just source inspection
// -- the pattern text only ever exists in macro-generated log lines, never
// literally in any .cpp file). The other 6 needed updating: legacy's
// snake_case function names (process_thread/calib_process/calib_reset/
// wrongway_check/wrongway_process/loadarea_process) all became camelCase
// methods during the port (plannerTick/calibProcess/calibReset/
// wrongwayCheck/wrongwayProcess/loadareaProcess) -- see
// 01_architecture.md's telemetry_node section for the full audit table.
// ---------------------------------------------------------------------------

static const FindPattern kFindPattern[TelemetryNode::FIND_PATTERN_COUNT] = {
    { "saveImage(): camera_image=", nullptr, 0, FIND_FLAGS_CAMERA_IMG },
    { "saveImage(): vision_image=", nullptr, 0, FIND_FLAGS_VISION_IMG },
    { "saveImage(): lidar_image=",  nullptr, 0, FIND_FLAGS_LIDAR_IMG },
    { "saveImage(): wmgrid_image=", nullptr, 0, FIND_FLAGS_WMGRID_IMG },
    { "saveImage(): navmap_image=", nullptr, 0, FIND_FLAGS_NAVMAP_IMG },
    { "saveImage(): cdepth_image=", nullptr, 0, FIND_FLAGS_CDEPTH_IMG },
    { "saveImage(): rcamera_image=", nullptr, 0, FIND_FLAGS_CAMERA2_IMG },
    { "saveImage(): rcdepth_image=", nullptr, 0, FIND_FLAGS_CDEPTH2_IMG },
    { "saveImage(): rvision_image=", nullptr, 0, FIND_FLAGS_VISION2_IMG },
    { "plannerTick(): process_angle", nullptr, 0, 0 },         // was "process_thread(): process_angle"
    { "gps_writeData(): fix=", nullptr, 0, 0 },
    { "process_readData(): process_change", nullptr, 0, 0 },
    { "INFO", "calibProcess", -1, 0 },                         // was "INFO"/"calib_process"
    { "calibReset", nullptr, 0, 0 },                           // was "calib_reset"
    { "INFO", "wrongwayCheck", -1, 0 },                        // was "INFO"/"wrongway_check"
    { "wrongwayProcess", nullptr, 0, 0 },                      // was "wrongway_process"
    { "navigation point passed", nullptr, 0, 0 },
    { "navig::navigation", nullptr, 0, 0 },
    { "loadareaProcess", nullptr, 0, 0 },                      // was "loadarea_process"
    { "ControlBoard::write(): data=\"D", nullptr, 0, 0 }
};

// fixme: possible buffer overflow -- legacy's own comment, kept verbatim (1:1 port)
static char *strcpyEdq(char *dst, const char *src)
// strcpy with inserting escape characters for double quotes:  msg="wrongway" -> msg=\"wrongway\"
{
    char *res = dst;
    while (*src != 0) {
        if (*src == '"') {
            *(dst++) = '\\';
        }
        *(dst++) = *(src++);
    }
    *dst = 0;
    return res;
}

// ---------------------------------------------------------------------------

TelemetryNode::TelemetryNode() : rclcpp::Node("telemetry_node")
{
    log_fname_ = LOG_FNAME;

    this->declare_parameter<std::string>("output_dir", "ramdisk/");
    output_dir_ = this->get_parameter("output_dir").as_string();
    if (!output_dir_.empty() && output_dir_.back() != '/') {
        output_dir_ += '/';
    }

    std::error_code ec;
    fs::create_directories(output_dir_, ec);
    if (ec) {
        LOGM_ERROR(loggerTelemetry, "TelemetryNode", "msg=\"could not create output_dir!\", output_dir=\""
            << output_dir_ << "\", error=\"" << ec.message() << "\"");
    }

    resetState();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(TELEMETRY_TICK_PERIOD),
        std::bind(&TelemetryNode::tick, this));

    LOGM_INFO(loggerTelemetry, "TelemetryNode", "msg=\"start\", log_fname=\"" << log_fname_
        << "\", output_dir=\"" << output_dir_ << "\", tick_period=" << TELEMETRY_TICK_PERIOD);
}

TelemetryNode::~TelemetryNode()
{
    if (fd_ != nullptr) {
        fclose(fd_);
        fd_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// resetState() -- 1:1 port of legacy init().
// ---------------------------------------------------------------------------

void TelemetryNode::resetState(void)
{
    buffer_cnt_ = 0;
    off_ = 0;
    read_empty_ = 0;
    for (int i = 0; i < FIND_PATTERN_COUNT; i++) {
        findResult_[i][0] = 0;
    }
}

// ---------------------------------------------------------------------------
// processLine() -- 1:1 port of legacy process_line(), unchanged algorithm
// (only kFindPattern[]/findResult_ renamed to match this port's naming).
// ---------------------------------------------------------------------------

void TelemetryNode::processLine(const char *p, size_t n)
{
    char line[LINE_SIZE + 1];
    if (n > (size_t)LINE_SIZE) {
        n = LINE_SIZE;
    }

    memcpy(line, p, n);
    line[n] = 0;

    for (int i = 0; i < FIND_PATTERN_COUNT; i++) {
        const char *ss;
        char *pl = line;
        int found = 0;
        do {
            ss = strstr(pl, kFindPattern[i].str1);
            if (ss == NULL) {
                pl = NULL;
                continue;  // not found
            }
            if (kFindPattern[i].str2 == NULL) {
                found = 1;
                continue;  // found "str1"
            }
            if (kFindPattern[i].str2 != NULL) {
                if (kFindPattern[i].dd >= 0) {
                    int ll1 = strlen(kFindPattern[i].str1) + kFindPattern[i].dd;
                    int ll2 = strlen(kFindPattern[i].str2);
                    if ((&(ss[ll1 + ll2]) - line) <= (int)n) {
                        int rr = memcmp(&(ss[ll1]), kFindPattern[i].str2, ll2);
                        if (rr == 0) {
                            found = 1;
                            continue;  // found "str1...str2"
                        }
                    }
                } else {
                    const char *ss2;
                    ss2 = strstr(ss, kFindPattern[i].str2);
                    if (ss2 != NULL) {
                        found = 1;
                        continue;  // found "str1.*str2"
                    }
                }
            }
            pl = const_cast<char *>(ss);
            pl++;
        } while ((pl != NULL) && (found == 0));
        if (found) {
            if (kFindPattern[i].flags == 0) {
                strcpy(&(findResult_[i][0]), line);
            } else {
                int ll = strlen(kFindPattern[i].str1);
                if (ss[ll] == '"') {
                    const char *ss2;
                    ss2 = strstr(&(ss[ll + 1]), "\"");
                    if (ss2 != NULL) {
                        int ll2 = ss2 - ss - ll - 1;
                        memcpy(&(findResult_[i][0]), &(ss[ll + 1]), ll2);
                        findResult_[i][ll2] = 0;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// writeResult() -- 1:1 port of legacy writeResult(). Writes to output_dir_
// (ROS parameter, default "ramdisk/") instead of legacy's hardcoded
// /var/www/html/ramdisk/ -- see class comment.
// ---------------------------------------------------------------------------

void TelemetryNode::writeResult(void)
{
    FILE *f;

    std::string txtFname  = output_dir_ + "istro_" EVENT_TAG "_out.txt";
    std::string htmlFname = output_dir_ + "istro_" EVENT_TAG "_out.html";
    std::string jsonFname = output_dir_ + "istro_" EVENT_TAG "_out.json";

    /* write TXT */
    f = fopen(txtFname.c_str(), "w");
    if (f == NULL) {
        LOGM_ERROR(loggerTelemetry, "writeResult", "msg=\"error opening output file!\", fname=\"" << txtFname << "\"");
        return;
    }
    for (int i = 0; i < FIND_PATTERN_COUNT; i++) {
        fprintf(f, "%s\n", findResult_[i]);
    }
    fclose(f);

    /* write HTML */
    f = fopen(htmlFname.c_str(), "w");
    if (f == NULL) {
        LOGM_ERROR(loggerTelemetry, "writeResult", "msg=\"error opening output file!\", fname=\"" << htmlFname << "\"");
        return;
    }
    fprintf(f, "<!DOCTYPE html>\n<html>\n<body>\n");
    fprintf(f, "<p style=\"font-family:Courier; font-size: 20px\">\n");
    for (int i = 0; i < FIND_PATTERN_COUNT; i++) {
        fprintf(f, "%s<br>\n", findResult_[i]);
    }
    fprintf(f, "</p>\n</body>\n</html>\n");
    fclose(f);

    /* write JSON */
    f = fopen(jsonFname.c_str(), "w");
    if (f == NULL) {
        LOGM_ERROR(loggerTelemetry, "writeResult", "msg=\"error opening output file!\", fname=\"" << jsonFname << "\"");
        return;
    }

    fprintf(f, "{\n");
    for (int i = 0; i < FIND_PATTERN_COUNT; i++) {
        if (findResult_[i][0] != 0) {
            if (kFindPattern[i].flags == FIND_FLAGS_CAMERA_IMG) {
                fprintf(f, "  \"camera_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_VISION_IMG) {
                fprintf(f, "  \"vision_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_LIDAR_IMG) {
                fprintf(f, "  \"lidar_image\":  \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_WMGRID_IMG) {
                fprintf(f, "  \"wmgrid_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_NAVMAP_IMG) {
                fprintf(f, "  \"navmap_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_CDEPTH_IMG) {
                fprintf(f, "  \"cdepth_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_CAMERA2_IMG) {
                fprintf(f, "  \"rcamera_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_CDEPTH2_IMG) {
                fprintf(f, "  \"rcdepth_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            } else
            if (kFindPattern[i].flags == FIND_FLAGS_VISION2_IMG) {
                fprintf(f, "  \"rvision_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult_[i]);
            }
        }
    }
    fprintf(f, "  \"items\": [\n");
    int found = 0;
    char line[LINE_SIZE + 1];
    for (int i = 0; i < FIND_PATTERN_COUNT; i++) {
        if (findResult_[i][0] != 0) {
            if (kFindPattern[i].flags == 0) {
                if (found) {
                    fprintf(f, ",\n");
                }
                found = 1;
                fprintf(f, "    \"%s\"", strcpyEdq(line, findResult_[i]));
            }
        }
    }
    fprintf(f, "\n  ]\n}\n");

    fclose(f);
}

// ---------------------------------------------------------------------------
// rotateLog() -- "copytruncate": copy the live shared log aside to a
// timestamped name (same naming convention conf/log4cxx.xml's own
// TimeBasedRollingPolicy used to produce, "istro_<EVENT_TAG>.HH_mm.log"), then
// truncate the live file to 0 in place (same inode -- NOT a rename). Every
// writer (camera_node x2, planner_node, ctrlboard_node, save_node,
// telemetry_node itself) opened log_fname_ in log4cxx append mode, which
// always writes at the current end of file rather than an internally
// remembered offset, so truncating in place is safe: their next write just
// lands at the new (zero) end of file, no reopen needed on their part. This
// replaces each process's own independent file-level rotation, which raced
// on renaming the same shared filename (see conf/log4cxx.xml's comment).
//
// GZIP_CMD's own glob ("istro_<EVENT_TAG>.*.log") picks up the file this creates
// on its own next GZIP_PERIOD tick, unchanged -- rotateLog() only creates the
// plain rotated .log, it does not gzip it itself.
// ---------------------------------------------------------------------------

void TelemetryNode::rotateLog(void)
{
    time_t rawtime;
    struct tm tm_now;
    time(&rawtime);
    localtime_r(&rawtime, &tm_now);

    char suffix[16];
    strftime(suffix, sizeof(suffix), "%H_%M", &tm_now);

    std::string rotated_name = "logout/istro_" EVENT_TAG "." + std::string(suffix) + ".log";

    std::error_code ec;
    fs::copy_file(log_fname_, rotated_name, fs::copy_options::overwrite_existing, ec);
    if (ec) {
        LOGM_ERROR(loggerTelemetry, "rotateLog", "msg=\"copy_file failed!\", rotated_name=\""
            << rotated_name << "\", error=\"" << ec.message() << "\"");
        return;
    }

    fs::resize_file(log_fname_, 0, ec);
    if (ec) {
        LOGM_ERROR(loggerTelemetry, "rotateLog", "msg=\"resize_file (truncate) failed!\", log_fname=\""
            << log_fname_ << "\", error=\"" << ec.message() << "\"");
        return;
    }

    // Realign our own tailing state to the now-empty live file -- proactive,
    // so we don't have to wait out the reactive 10-empty-read resetState()
    // path (tick()'s own comment) to notice. findResult_[] is deliberately
    // left untouched (unlike resetState()) so the dashboard keeps showing its
    // last known values through the rotation instant instead of blanking out
    // for a couple seconds every minute.
    if (fd_ != nullptr) {
        fclose(fd_);
        fd_ = nullptr;
    }
    off_ = 0;
    buffer_cnt_ = 0;
    read_empty_ = 0;

    LOGM_INFO(loggerTelemetry, "rotateLog", "msg=\"rotated\", rotated_name=\"" << rotated_name << "\"");
}

// ---------------------------------------------------------------------------
// tick() -- own rclcpp::WallTimer (TELEMETRY_TICK_PERIOD=250ms, matching
// legacy's own msleep(250)), replacing legacy main()'s own while(true) loop.
// 1:1 port of that loop's body, minus the sleep itself (the timer paces
// this instead) and the outer "while(true)" (each tick() call is one
// iteration).
// ---------------------------------------------------------------------------

void TelemetryNode::tick(void)
{
    /* open & seek */
    if (fd_ == nullptr) {
        fd_ = fopen(log_fname_.c_str(), "rb");
        if (fd_ == nullptr) {
            resetState();
            LOGM_TRACE(loggerTelemetry, "tick", "msg=\"open: error: could not open file!\", log_fname=\"" << log_fname_ << "\"");
        } else
        if (off_ > 0) {
            if (fseek(fd_, off_, SEEK_SET) != 0) {
                LOGM_ERROR(loggerTelemetry, "tick", "msg=\"lseek: fseek error\"");
                fclose(fd_);
                fd_ = nullptr;
                resetState();
            }
        }
    }

    /* read & process */
    if (fd_ != nullptr) {
        int buffer_full = 0;
        do {
            /* read next bytes until end of buffer or until end of file */
            size_t read_cnt;
            read_cnt = fread(&(buffer_[buffer_cnt_]), 1, BUFFER_SIZE - buffer_cnt_, fd_);
            if (read_cnt == 0) {
                read_empty_++;
            }
            // read_empty_ is deliberately never reset on a successful
            // (nonzero) read -- NOT a bug (user correction, this was
            // initially mis-flagged as one): fread() returning 0 means
            // either "genuine read error" or "at EOF, nothing new yet (the
            // completely normal tailing case)", and there is no way to
            // distinguish the two from the return value alone. The design
            // instead leans on an assumption about the log's own normal
            // volume -- hundreds of lines/second across all nodes during
            // real operation -- so 10 *consecutive* empty reads (~2.5s at
            // this node's own 250ms period) is itself the signal something
            // is wrong (file rotated/truncated/replaced under us, or the
            // whole application stopped), safely handled either way by the
            // same cheap response: close, reset, and re-read from the top
            // next tick. Self-corrects within one tick even when nothing was
            // actually wrong (the next do-while re-reads the whole file and
            // repopulates findResult_[] identically, since the scan is
            // deterministic) -- a viewer of istro_<EVENT_TAG>_out.txt never sees
            // it, and on this dev VM's much quieter test logs it triggers
            // more often than it would against the real robot's own log
            // volume, but that's this environment being quiet, not the
            // mechanism being wrong.
            // Legacy's own condition also has "(read_cnt < 0) ||" here --
            // read_cnt is size_t (unsigned), so that comparison is always
            // false (dead code -- also flagged by this project's own
            // -Wextra/-Wtype-limits, CMakeLists.txt) and could never have
            // actually detected a real read error, unlike what its name
            // suggests. Replaced with ferror(fd_) -- the actual, standard
            // way to tell a genuine I/O error apart from fread() simply
            // returning less than requested because it hit EOF (ferror()'s
            // error indicator is only set by a real failed read, never by
            // reaching end-of-file alone; feof() is the separate indicator
            // for that). Not a redesign of read_empty_'s own logic above --
            // this only replaces the one condition that was never doing
            // anything.
            if (ferror(fd_) || (read_cnt > BUFFER_SIZE - buffer_cnt_) || (read_empty_ > 10)) {
                if (ferror(fd_)) {
                    LOGM_ERROR(loggerTelemetry, "tick", "msg=\"read: could not read from file!\", error=\"" << strerror(errno) << "\"");
                } else {
                    LOGM_ERROR(loggerTelemetry, "tick", "msg=\"read: error2: could not read from file!\"");
                }
                read_cnt = 0;
                fclose(fd_);
                fd_ = nullptr;
                resetState();
                break;
            }

            off_ += read_cnt;
            buffer_cnt_ += read_cnt;
            buffer_full = (buffer_cnt_ == BUFFER_SIZE);

            const char *pb = buffer_;

            size_t n = buffer_cnt_;
            while (n > 0) {
                size_t nl;
                const char *pl;

                pl = (const char *)memchr(pb, '\n', n);
                if (pl == NULL)
                    break;

                nl = pl - pb;  // chars to be processed

                if ((nl > 0) && (*(pl - 1) == '\r')) {
                    processLine(pb, nl - 1);
                } else {
                    processLine(pb, nl);
                }

                pb = pl;
                pb++;
                n -= nl + 1;
            }

            // line end not found in buffer? ignore all characters
            if (n == BUFFER_SIZE) {
                buffer_cnt_ = 0;
            } else
            // some unprocessed data left?
            if (n > 0) {
                buffer_cnt_ = n;
                memcpy(buffer_, pb, n);
            } else {
                buffer_cnt_ = 0;
            }
        } while (buffer_full);
    }

    /* write, close */
    if (fd_ != nullptr) {
        writeResult();
        fclose(fd_);
        fd_ = nullptr;
    }

    rotate_sleept_ += TELEMETRY_TICK_PERIOD;
    if (rotate_sleept_ >= ROTATE_PERIOD) {
        rotateLog();
        rotate_sleept_ = 0;
    }

    sleept_ += TELEMETRY_TICK_PERIOD;
    if (sleept_ >= GZIP_PERIOD) {
        LOGM_INFO(loggerTelemetry, "tick", "msg=\"system: gzip executed...\"");
        system(GZIP_CMD);
        sleept_ = 0;
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Every other node gets this from its own istro_main_<subsystem>()
    // (called from main() before constructing the Node, same placement
    // here); telemetry_node has no such scaffold to call (see class
    // comment), so it's inlined directly -- without it, log4cxx has no
    // appender configured for any LOGM_*/LOG_* call at all ("No appender
    // could be found for logger", found via testing, not by inspection).
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerTelemetry, "-----------------------------");
    LOGM_INFO(loggerTelemetry, "main", "msg=\"application start\"");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
