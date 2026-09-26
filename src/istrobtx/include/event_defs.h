#ifndef __EVENT_DEFS_H__
#define __EVENT_DEFS_H__

// Single source of truth for the current competition's short file-naming
// tag, used to build output filenames across istrorsx_core (save_node.cpp's
// saveImage()/saveNavMap(), telemetry_node.cpp's LOG_FNAME/GZIP_CMD/output
// filenames) and matched by conf/log4cxx.xml's own <param name="file"/>
// FileNamePattern -- change EVENT_TAG here and every one of those C++ call
// sites picks it up automatically; conf/log4cxx.xml is a separate XML file
// the C preprocessor can't reach, so its log file name (and the comment above
// it) still needs updating by hand alongside this header. Shell/Python scripts
// read the tag from this file instead (script/navmap_export.sh) or accept any
// tag (script/log/wrongway_analyze.py).
//
// Named EVENT_TAG, not RT_TAG/ROBOTOUR_TAG -- "RT" specifically means
// Robotour, but this project's robot also competes at events with different
// short codes (RR = Robotem Rovne, RO = RoboOrienteering, ...), so the
// letters themselves change across events, not just the year.
//
// Deliberately NOT used for comments/documentation referencing the legacy
// source file istro_rt2025.cpp (script/logp.cpp's own equivalent, etc.) --
// that's a historical filename, this repo's own permanent reference copy of
// a specific past codebase snapshot, and stays istro_rt2025.cpp regardless
// of which competition this constant is currently set for.
#define EVENT_TAG "rt2026"

#endif
