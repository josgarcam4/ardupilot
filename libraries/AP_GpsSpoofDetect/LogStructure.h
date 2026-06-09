// GPS Spoof Detector Log Message Definition

// Log structure for GSPF (GPS Spoofing Detection)
struct PACKED log_GSPF {
    LOG_PACKET_HEADER;
    uint64_t time_us;     // Time since system start
    uint8_t state;        // NOMINAL=0, SUSPICIOUS=1, CONFIRMED=2
    float score;          // Combined spoof detection score [0, 2]
    float f1_score;       // Feature 1: Innovation persistence
    float f2_score;       // Feature 2: Velocity coherence
    float f3_score;       // Feature 3: Acceleration consistency
    float f4_score;       // Feature 4: Innovation whiteness
    uint8_t feature_flags; // Flags indicating which features are elevated
    uint8_t reserved;
};

#define LOG_GSPF_MSG_LEN sizeof(struct log_GSPF)
#define LOG_GSPF_MSGID 0xFF  // Placeholder, typically assigned by AP_Logger

// GPS Spoof Detector Log Catalog Entry
// Add this to AP_Logger's log catalog:
// { LOG_GSPF_MSG_LEN, LOG_GSPF_MSGID, "GSPF", "QBfffffBB",
//   "TimeUS,State,Score,F1,F2,F3,F4,Flags,Res" }
