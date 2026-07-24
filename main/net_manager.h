#pragma once

#include <stdbool.h>

// Fires once, the first time any network interface gets a usable IP.
typedef void (*net_manager_ready_cb_t)(void);

// Fires on every subsequent change of the active interface (Wi-Fi/cellular
// failover). Not fired for the initial connect.
typedef void (*net_manager_changed_cb_t)(void);

void net_manager_init(net_manager_ready_cb_t on_ready, net_manager_changed_cb_t on_changed);
bool net_manager_is_connected(void);

// Forces a reset of whichever interface is currently active: Wi-Fi
// disconnect/reconnect, or a cellular modem teardown/restart.
void net_manager_reset(void);
