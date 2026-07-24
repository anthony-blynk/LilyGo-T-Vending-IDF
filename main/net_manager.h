#pragma once

#include <stdbool.h>

// Fires once, the first time any network interface gets a usable IP.
typedef void (*net_manager_ready_cb_t)(void);

// Fires on every subsequent change of the active interface (e.g. Wi-Fi/cellular
// failover, once a second interface exists). Not fired for the initial connect.
typedef void (*net_manager_changed_cb_t)(void);

void net_manager_init(net_manager_ready_cb_t on_ready, net_manager_changed_cb_t on_changed);
bool net_manager_is_connected(void);

// Forces the Wi-Fi station to drop and re-associate.
void net_manager_reset(void);
