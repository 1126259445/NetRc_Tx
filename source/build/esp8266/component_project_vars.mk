# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/include /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/esp8266 -lesp8266 -L/home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib -lgcc -lhal -lcore -lnet80211 -lphy -lrtc -lclk -lpp -lsmartconfig -lssc -lwpa -lespnow -lwps -lwpa2 -L /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/ld -T esp8266_out.ld -T $(BUILD_DIR_BASE)/esp8266/esp8266.project.ld -Wl,--no-check-sections -u call_user_start -u g_esp_sys_info -T esp8266.rom.ld -T esp8266.peripherals.ld
COMPONENT_LINKER_DEPS += /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libgcc.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libhal.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libcore.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libnet80211.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libphy.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/librtc.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libclk.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libpp.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libsmartconfig.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libssc.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libwpa.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libespnow.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libwps.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/lib/libwpa2.a /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/ld/esp8266.rom.ld /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/ld/esp8266.peripherals.ld $(BUILD_DIR_BASE)/esp8266/esp8266.project.ld
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += esp8266
COMPONENT_LDFRAGMENTS += /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/ld/esp8266_fragments.lf /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/ld/esp8266_bss_fragments.lf /home/xc/ESP/NetRc/NetRc_Tx/components/esp8266/linker.lf
component-esp8266-build: 
