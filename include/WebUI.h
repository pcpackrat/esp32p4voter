#ifndef WEB_UI_H
#define WEB_UI_H

// Start the web server. Call once, after any network interface (ETH, WiFi or
// setup hotspot) is up.
void webBegin();
void webHandle();

#endif
