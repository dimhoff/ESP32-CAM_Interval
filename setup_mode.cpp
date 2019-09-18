/**
 * setup_mode.cpp - Set-Up mode, Wi-Fi AP and web server
 *
 * Copyright (c) 2019, David Imhoff <dimhoff.devel@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "config.h"

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>

#include "configuration.h"
#include "camera.h"
#include "html_content.h"
#include "io_defs.h"

#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

static const byte DNS_PORT = 53;
static const IPAddress SERVER_IP(192, 168, 1, 1);
static const IPAddress GATEWAY_IP(0, 0, 0, 0);
static const IPAddress SERVER_NETMASK(255, 255, 255, 0);
static const int WIFI_DEFAULT_CHANNEL = 6;

DNSServer dnsServer;
WebServer webServer(80);

static void send_compressed(PGM_P content, size_t content_len, PGM_P mime)
{
  webServer.sendHeader("Content-Encoding", "gzip");
  webServer.send_P(200, mime, content, content_len);
}

void httpHandleImage()
{
  camera_fb_t *fb;

  fb = camera_capture();

  // FIXME: *_P() functions require buf to be DWORD aligned!! It probably is. (is this also required for the DMA engine that writes to buf?)
  webServer.send_P(200, "image/jpeg", (const char *) fb->buf, fb->len);

  camera_fb_return(fb);
}

void httpHandleApply()
{
  if (cfg.saveConfig()) {
    webServer.send(200, "application/json", "{\"success\":true}");
  } else {
    webServer.send(200, "application/json", "{\"success\":false, \"error\":\"Failed to save config\"}");
  }
}

void httpHandleRestart()
{
  // No reply since the reply won't make it to the client before reboot...
  // TODO: can we send a reply and then close the connection or de-init server to make sure it is received?
  esp_restart();
}

void httpHandleSet()
{
  if (!webServer.hasArg("key") || !webServer.hasArg("val")) {
    webServer.send(200, "application/json", "{\"success\":false,\"error\":\"Missing argument\"}");
    return;
  }

  if (cfg.config_set(webServer.arg("key").c_str(), webServer.arg("val").c_str()) != 0) {
    webServer.send(200, "application/json", "{\"success\":false,\"error\":\"Invalid key or value\"}");
    return;
  }

  if (camera_reconfigure() != true) {
    webServer.send(200, "application/json",
        "{\"success\":false,\"error\":\"Failed to reconfigure camera\"}");
    return;
  }

  webServer.send(200, "application/json", "{\"success\":true}");
}

void httpHandleSetTime()
{
  String timeStr = webServer.arg("time");
  size_t msPos = timeStr.length() - 3;

  struct timeval now = {
    timeStr.substring(0, msPos).toInt(),
    timeStr.substring(msPos).toInt() * 1000
  };

  if (now.tv_sec < NOT_BEFORE_TIME) {
    webServer.send(200, "application/json",
        "{\"success\":false,\"error\":\"Invalid time value\"}");
    return;
  }

  if (settimeofday(&now, NULL) != 0) {
    webServer.send(200, "application/json",
        "{\"success\":false,\"error\":\"Failed to set time\"}");
    // TODO: append strerror to error string
    return;
  }

  webServer.send(200, "application/json", "{\"success\":true}");
}

void httpHandleConfig()
{
  webServer.send(200, "application/json", cfg.configAsJSON());
}

void httpHandleRoot()
{
  send_compressed(content_index_html, content_len_index_html, PSTR("text/html"));
}

void httpHandleTzinfo()
{
  send_compressed(content_tzinfo_json, content_len_tzinfo_json, PSTR("application/json"));
}

bool setup_mode_init()
{
  // Start Wi-FI
  Serial.println("Setting up Wi-Fi AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(SERVER_IP, GATEWAY_IP, SERVER_NETMASK);
  WiFi.softAP("ESP32-interval Set-up mode", "", WIFI_DEFAULT_CHANNEL, false, 1);

  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request
  Serial.println("Staring DNS server");
  dnsServer.start(DNS_PORT, "*", SERVER_IP);

  // Config HTTPd
  Serial.println("Staring Web server");
  webServer.on("/", httpHandleRoot);
  webServer.on("/image.jpg", httpHandleImage);
  webServer.on("/tzinfo.json", httpHandleTzinfo);
  webServer.on("/apply", httpHandleApply);
  webServer.on("/restart", httpHandleRestart);
  webServer.on("/set", httpHandleSet);
  webServer.on("/set_time", httpHandleSetTime);
  webServer.on("/config", httpHandleConfig);
  webServer.onNotFound(httpHandleRoot);
  webServer.begin();

  return true;
}

void setup_mode_loop()
{
  const static long blink_sequence[] = { 100, 200, 100, 1600};
  static long blink_last = 0;
  static uint8_t blink_idx = 0;

  // Blink LED
  long now = millis();
  if (now - blink_last > blink_sequence[blink_idx]) {
    blink_idx = (blink_idx + 1) % ARRAY_SIZE(blink_sequence);
    blink_last = now;
    digitalWrite(LED_GPIO_NUM, (blink_idx & 1) ? HIGH : LOW);
  }

  dnsServer.processNextRequest();
  webServer.handleClient();
}
