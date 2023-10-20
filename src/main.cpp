#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <FT6336.h>
#include <Audio.h>

#include "gui/ui.h"

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

#define LCD_BL_PIN (3)

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

#define I2C_SDA_PIN (15)
#define I2C_SCL_PIN (16)

TFT_eSPI tft = TFT_eSPI(); /* TFT instance */

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t * data) {
  uint8_t touchPoint = touch.read((uint16_t*)(&data->point.x), (uint16_t*)(&data->point.y));
  if (touchPoint > 0) {
    // Serial.printf("X: %d\tY: %d\n", data->point.x, data->point.y);
  }
  data->state = touchPoint > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

#define I2S_DOUT      47
#define I2S_BCLK      48
#define I2S_LRC       45

Audio audio;
String ssid =     "Artron@Kit";
String password = "Kit_Artron";

void on_station_click_cb(lv_event_t * e) {
  lv_obj_t * target = lv_event_get_target(e);

  lv_obj_t * radio_select_btn[] = {
      ui_cool93,
      ui_green_wave,
      ui_great_93_fm,
      ui_efm
    };
  for (uint8_t i=0;i<(sizeof(radio_select_btn) / sizeof(lv_obj_t*));i++) {
    lv_obj_clear_state(radio_select_btn[i], LV_STATE_CHECKED);
  }

  lv_obj_add_state(target, LV_STATE_CHECKED);

  if (target == ui_cool93) {
    audio.connecttohost("https://coolism-web.cdn.byteark.com/;stream/1");
  } else if (target == ui_green_wave) {
    audio.connecttohost("https://atimeonline3.smartclick.co.th/green");
  } else if (target == ui_great_93_fm) {
    audio.connecttohost("https://radio.vpsthai.net/proxy/qndrbfic/stream");
  } else if (target == ui_efm) {
    audio.connecttohost("https://atimeonline3.smartclick.co.th/efm");
  }
  lv_obj_add_state(ui_play_stop_btn, LV_STATE_CHECKED);
}

void setup() {
    Serial.begin( 115200 ); /* prepare for possible serial debug */

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    lv_init();

    tft.begin();          /* TFT init */
    tft.setRotation( 1 ); /* Landscape orientation, flipped */
    // tft.invertDisplay(false);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400E3);
    touch.init();

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
    
    ui_init();
    
    lv_obj_add_event_cb(ui_volume, [](lv_event_t * e) {
      int new_volume = lv_slider_get_value(ui_volume);
      audio.setVolume(map(new_volume, 0, 100, 0, 21));
    }, LV_EVENT_VALUE_CHANGED, NULL);

    static lv_obj_t * radio_select_btn[] = {
      ui_cool93,
      ui_green_wave,
      ui_great_93_fm,
      ui_efm
    };

    for (uint8_t i=0;i<(sizeof(radio_select_btn) / sizeof(lv_obj_t*));i++) {
      lv_obj_add_event_cb(radio_select_btn[i], on_station_click_cb, LV_EVENT_CLICKED, NULL);
    }
    
    lv_obj_add_event_cb(ui_play_stop_btn, [](lv_event_t * e) {
      audio.pauseResume();
      /*
      if (lv_obj_has_state(ui_play_stop_btn, LV_STATE_CHECKED)) {
        audio.stopSong();
        lv_obj_clear_state(ui_play_stop_btn, LV_STATE_CHECKED);
      }
      */
    }, LV_EVENT_CLICKED, NULL);

    pinMode(LCD_BL_PIN, OUTPUT);
    digitalWrite(LCD_BL_PIN, HIGH);

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
}

void loop() {
    lv_timer_handler(); /* let the GUI do its work */
    audio.loop();
    // delay(5);
}
