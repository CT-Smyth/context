// ======================================================================================NETWORK 


void startWiFi(){
  // Automatically connect using saved credentials,
  WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP

  WPASS = wm.getWiFiPass();
  WSSID = wm.getWiFiSSID();

  if (WSSID != "") {
    Serial.print("\nConnect Wifi to:");
    Serial.print(WSSID);
    Serial.print("  Using:");
    Serial.println(WPASS);
    WiFi.begin(WSSID, WPASS);
  } else {
    Serial.println("No credentials found. use double reset or hold down action button");
    Serial.println("during boot to Launch config portal");
  }
  //const char *HOST_NAME = "flowmeter";  //set hostname to <hostname>.local
  const char *HOST_NAME = WiFi.getHostname();

#ifdef MDNS_RESPONDER
  if (!MDNS.begin(HOST_NAME)) {  // Set the hostname to "esp32.local"
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
#endif

#ifdef STATIC_IP
  // Configuring static IP
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure Static IP");
  }
#endif
}


void launchPortal(){
  digitalWrite(ledPin, ledOn);

    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(300);
    wm.startConfigPortal(WiFi.getHostname(), "configureMe!"); //place credentials here.

}