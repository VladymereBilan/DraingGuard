// Wi-Fi and backend credentials.
//
// This header is the only place in the project that touches secrets.h, which
// keeps config.h free of secrets and Wi-Fi logic (decision D24). It lives under
// src/ because only hardware code needs credentials - lib/dg_core must never
// include it.

#ifndef DG_NET_CONFIG_H
#define DG_NET_CONFIG_H

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h and fill it in"
#endif

namespace net {

#ifdef DG_WOKWI
// Wokwi's simulated access point: open network, empty password.
inline constexpr char WIFI_SSID_STR[] = "Wokwi-GUEST";
inline constexpr char WIFI_PASS_STR[] = "";
#else
inline constexpr char WIFI_SSID_STR[] = WIFI_SSID;
inline constexpr char WIFI_PASS_STR[] = WIFI_PASS;
#endif

// ThingSpeak target. Real credentials even under DG_WOKWI: the simulator has
// outbound network access, so uploads can be exercised there too (D9 pending).
inline constexpr char TS_CHANNEL_ID_STR[] = TS_CHANNEL_ID;
inline constexpr char TS_WRITE_KEY_STR[] = TS_WRITE_KEY;

} // namespace net

#endif // DG_NET_CONFIG_H
