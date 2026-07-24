# Network interface failover — lessons from blynk_p4_mqtt (Wi-Fi/Ethernet)

Written for reuse on the ESP32-S3 + SIM7000 (cellular) + RS485 project, which is
moving from Arduino to ESP-IDF and wants Wi-Fi/cellular failover analogous to
this project's Wi-Fi/Ethernet failover. **Don't copy `net_manager.c` verbatim** —
PPP/cellular has different bring-up timing and event shapes than Ethernet (no
physical link-up/down signal, dial-up delay, signal-quality concerns). Rewrite a
fresh `net_manager` for that project following the *pattern* below, verifying
each esp-idf/esp_modem detail against that project's actual headers rather than
assuming it matches Ethernet's behavior.

Everything below was verified against ESP-IDF v6.0.2 source during that
project's development (`esp_netif_lwip.c`, `esp_eth_netif_glue.c`,
`mqtt_client.c`, `esp_netif_defaults.h`) — not guessed. Where cellular specifics
are unverified, it's called out explicitly.

## The architecture that worked

A `net_manager` module owns bringing up all network interfaces and exposes two
callbacks to the app:
- `on_ready` — fires once, the first time *any* interface gets a usable IP. App
  starts its MQTT/application client here.
- `on_changed` — fires every subsequent time the *active default* interface
  changes. App calls something like `mqtt_client_force_reconnect()` here,
  because an already-open TCP/TLS session does not follow the OS's routing
  table when it changes — it keeps using whatever local interface it was
  opened on until explicitly torn down.

This cleanly separates "which interface should carry traffic" (esp_netif's job)
from "does the app need to know" (net_manager's job) from "what does the app do
about it" (the app's job, one function call).

## Gotcha 1: `route_prio` — higher number wins, not lower

Despite what you'd guess from the name, ESP-IDF's default-netif arbitration
(`esp_netif_update_default_netif_lwip()` in `esp_netif_lwip.c`) picks whichever
"up" interface has the **highest** `route_prio` value, not the lowest. Built-in
defaults (`esp_netif_defaults.h`):

| Interface | `route_prio` |
|---|---|
| Wi-Fi AP | 10 |
| Wi-Fi NAN | 10 |
| PPP | 20 |
| Ethernet | 50 |
| Bridge | 70 |
| Wi-Fi STA | 100 |

For this project (prefer Ethernet over Wi-Fi), Ethernet's default (50) is
*lower* than Wi-Fi STA's (100) — the opposite of what's wanted — so
`net_manager.c` explicitly overrides Ethernet's netif config to
`route_prio = 200` before creating it, to beat Wi-Fi's 100.

**For the S3/cellular project**: if the intent is "prefer Wi-Fi, fall back to
cellular," note that PPP's default (20) is already *lower* than Wi-Fi STA's
(100) — meaning Wi-Fi already wins by default with **no override needed at
all**. Only add an override if the desired priority order doesn't already fall
out of the built-in defaults. Don't assume this without checking the actual
priority you want against the table above.

## Gotcha 2: don't trust the "default netif changed" the instant you observe a link event

`esp_netif_get_default_netif()` can be checked from two different kinds of
places, with different safety:

- **GOT_IP events** (`IP_EVENT_STA_GOT_IP`, `IP_EVENT_ETH_GOT_IP`, presumably
  `IP_EVENT_PPP_GOT_IP` too, verify): esp_netif calls
  `esp_netif_update_default_netif()` **synchronously, before** posting the
  event (see `esp_netif_lwip.c` around the DHCP-success callback). By the time
  any handler processes the posted event, arbitration has already settled.
  Safe to check immediately.
- **Link up/down events** (`WIFI_EVENT_STA_DISCONNECTED`,
  `ETHERNET_EVENT_CONNECTED/DISCONNECTED`): the netif's own glue code (e.g.
  `esp_eth_netif_glue.c`) registers *its own* handler on this same event to
  call `esp_netif_up()`/`esp_netif_down()` (which is what actually re-runs the
  arbitration). Handlers for one event fire in **registration order** — so
  whether your own handler sees the update-to-date state depends on whether
  you registered before or after that glue handler, which itself depends on
  whether you registered before or after the netif was created.

This means:
- Registering your handler **before** creating the netifs → you never miss a
  one-shot connect/start event, but you can see **stale** default-netif state
  on link up/down events (arbitration hasn't run yet when you check).
  Ethernet's PHY bring-up in this project took several seconds *longer* than
  Wi-Fi's association+DHCP — registering **after** creating the netifs to fix
  the staleness caused the opposite bug: missing Wi-Fi's real one-shot
  `STA_CONNECTED`/`GOT_IP` events entirely, because they fired before
  registration completed. Neither order is safe on its own.
- **The fix that actually worked**: keep handler registration *before* netif
  creation (so no one-shot event is ever missed), and add a lightweight
  periodic poll (`esp_timer`, ~1s interval in this project) that just re-runs
  the same "check current default, notify if changed" logic on a timer. A poll
  can't miss a one-shot event because it doesn't rely on one, and it always
  eventually observes the settled state — it's a backstop for the cases the
  event-driven path gets stale/racy on, not a replacement for it.

**For cellular**, there's no physical link-up/down signal at all — PPP session
loss usually shows up as `IP_EVENT_PPP_LOST_IP` or a modem-library-specific
event, and may need active monitoring (e.g. periodic AT command / signal
quality check) rather than a hardware interrupt. The poll-based backstop
becomes even more valuable here, possibly the *primary* detection mechanism
rather than just a backstop — verify what events (if any) esp_modem actually
gives you for "PPP session dropped" before designing around it.

## Gotcha 3: gate "ready"/"changed" on an actual GOT_IP, not just "up"

`esp_netif_get_default_netif()` can elect an interface as default once its link
is administratively "up," which can happen well before DHCP (or PPP IPCP
negotiation) actually hands it a usable IP/DNS server. Notifying the app at
that point makes it try — and fail — a connection too early (observed
directly: an MQTT connect attempt failed with a DNS resolution error because it
fired right as Ethernet's link came up, before its DHCP lease completed).

Fix: track "has this specific netif actually seen its own GOT_IP event" per
interface (a `bool` per interface, set on GOT_IP, cleared on
disconnect/lost-IP), and only act on `esp_netif_get_default_netif()`'s result
if the netif it currently points to has that flag set. Otherwise silently
return and wait for the next event/poll tick.

## Gotcha 4: `esp_mqtt_client_reconnect()` is not "force a reconnect"

This one cost real debugging time. `esp_mqtt_client_reconnect()`
(`mqtt_client.c` in the `espressif/mqtt` managed component) only actually does
something if the client is *already* in `MQTT_STATE_WAIT_RECONNECT` — i.e.
already disconnected and waiting to retry:

```c
esp_err_t esp_mqtt_client_reconnect(esp_mqtt_client_handle_t client)
{
    ...
    if (client->state != MQTT_STATE_WAIT_RECONNECT) {
        ESP_LOGD(TAG, "The client is not waiting for reconnection. Ignore the request");
        return ESP_FAIL;
    }
    ...
}
```

It logs `"Client force reconnect requested"` at `ESP_LOGI` regardless — making
it *look* like it worked even when it silently no-ops. If the client is
currently `CONNECTED` (the exact situation when net_manager's `on_changed`
fires because a *better* interface just came up while the current session is
still healthy), this call does nothing, and the client keeps using its
existing (now suboptimal) session indefinitely.

The correct call for "force a healthy, connected client to drop and reconnect"
is **`esp_mqtt_client_disconnect()`**, which works from `CONNECTED` and (with
auto-reconnect enabled, the default) triggers the same automatic reconnect
machinery afterward. Also tune `network.reconnect_timeout_ms` down from the
10s default if you want failover to be visibly prompt rather than technically-
working-but-slow — 1000ms worked well here.

This same bug pattern existed in this project's `downlink/redirect` handler too
(calling `reconnect()` right after `esp_mqtt_client_set_uri()`, while the
client is always `CONNECTED` at that point since redirect only arrives over an
active session) — worth grepping for the same mistake in any MQTT-adjacent code
carried over from the Arduino codebase, if it uses esp-mqtt underneath.

## What will genuinely differ for Wi-Fi + cellular (verify, don't assume)

- **Bring-up time**: modem dial-up (AT command negotiation, network
  registration, signal search) is likely much slower and more variable than
  either Wi-Fi or Ethernet here — the has-IP gating and poll interval may need
  longer/adaptive timing.
- **"Is it actually usable" signal**: no physical link like Ethernet's;
  probably need to poll AT+CSQ (signal quality) or watch for esp_modem's own
  state events. Check what event base/IDs `espressif/esp_modem` actually
  exposes before designing the equivalent of `ETHERNET_EVENT_CONNECTED/
  DISCONNECTED` — don't assume it mirrors Ethernet's shape.
- **Cost/reliability of switching**: cellular data may be metered/slower to
  reestablish than Ethernet — consider whether flapping between Wi-Fi and
  cellular needs hysteresis (e.g. require cellular to be down for N seconds
  before actually failing over, so a momentary Wi-Fi hiccup doesn't trigger an
  expensive PPP redial) — Ethernet/Wi-Fi didn't need this because Ethernet
  reconnects are cheap and near-instant, cellular's aren't.
- **RS485**: unrelated to network failover, just a UART peripheral — no
  crossover with the lessons here.
