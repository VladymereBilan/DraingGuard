// Template for include/secrets.h.
//
// Copy this file to include/secrets.h and fill in the real values. secrets.h is
// gitignored and must never be committed.
//
//   copy firmware\include\secrets.example.h firmware\include\secrets.h
//
// The wokwi environment does not need real Wi-Fi credentials (it joins the open
// Wokwi-GUEST network), but secrets.h must still exist for the build to compile.

#ifndef DG_SECRETS_H
#define DG_SECRETS_H

#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASS "your-wifi-password"

// ThingSpeak channels (D28):
//   DrainGuard-dev  3568459   Wokwi and the fake writer - use this one for now
//   DrainGuard      3568466   real hardware and the evaluation trials
// Switch by editing this file; the firmware just uses TS_CHANNEL_ID.
#define TS_CHANNEL_ID "3568459"
#define TS_WRITE_KEY "YOURTHINGSPEAKWRITEKEY"

#endif // DG_SECRETS_H
