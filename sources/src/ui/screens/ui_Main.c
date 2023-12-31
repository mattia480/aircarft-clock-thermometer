// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: TermometroAereo

#include "../ui.h"

void ui_Main_screen_init(void)
{
    ui_Main = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Main, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Main, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Main, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SettingBtn = lv_btn_create(ui_Main);
    lv_obj_set_width(ui_SettingBtn, 240);
    lv_obj_set_height(ui_SettingBtn, 75);
    lv_obj_set_align(ui_SettingBtn, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_SettingBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_SettingBtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SettingBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SettingBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SettingIcon = lv_img_create(ui_Main);
    lv_img_set_src(ui_SettingIcon, &ui_img_settings_icon_png);
    lv_obj_set_width(ui_SettingIcon, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_SettingIcon, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SettingIcon, -5);
    lv_obj_set_y(ui_SettingIcon, 5);
    lv_obj_set_align(ui_SettingIcon, LV_ALIGN_TOP_RIGHT);
    lv_obj_add_flag(ui_SettingIcon, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_SettingIcon, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_PressureHumidityBtn = lv_btn_create(ui_Main);
    lv_obj_set_width(ui_PressureHumidityBtn, 240);
    lv_obj_set_height(ui_PressureHumidityBtn, 110);
    lv_obj_set_x(ui_PressureHumidityBtn, 0);
    lv_obj_set_y(ui_PressureHumidityBtn, 148);
    lv_obj_set_align(ui_PressureHumidityBtn, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_PressureHumidityBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_PressureHumidityBtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_PressureHumidityBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_PressureHumidityBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ResetRuntimeBtn = lv_btn_create(ui_Main);
    lv_obj_set_width(ui_ResetRuntimeBtn, 240);
    lv_obj_set_height(ui_ResetRuntimeBtn, 50);
    lv_obj_set_x(ui_ResetRuntimeBtn, 0);
    lv_obj_set_y(ui_ResetRuntimeBtn, 268);
    lv_obj_set_align(ui_ResetRuntimeBtn, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_ResetRuntimeBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_ResetRuntimeBtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_ResetRuntimeBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ResetRuntimeBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ModelLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_ModelLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ModelLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ModelLabel, 0);
    lv_obj_set_y(ui_ModelLabel, 5);
    lv_obj_set_align(ui_ModelLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_ModelLabel, "Alpi Aviation\nPioneer 300 Hawk");
    lv_obj_set_style_text_color(ui_ModelLabel, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ModelLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ModelLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ModelLabel, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_RegistrationLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_RegistrationLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_RegistrationLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_RegistrationLabel, 0);
    lv_obj_set_y(ui_RegistrationLabel, 45);
    lv_obj_set_align(ui_RegistrationLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_RegistrationLabel, "I-A931");
    lv_obj_set_style_text_font(ui_RegistrationLabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TimeHS = lv_label_create(ui_Main);
    lv_obj_set_width(ui_TimeHS, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TimeHS, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TimeHS, 0);
    lv_obj_set_y(ui_TimeHS, 65);
    lv_obj_set_align(ui_TimeHS, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_TimeHS, "____________________________________");

    ui_DateTimeLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_DateTimeLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_DateTimeLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_DateTimeLabel, 0);
    lv_obj_set_y(ui_DateTimeLabel, 85);
    lv_obj_set_align(ui_DateTimeLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_DateTimeLabel, "Date & Time (IT)");
    lv_obj_set_style_text_font(ui_DateTimeLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_DateText = lv_label_create(ui_Main);
    lv_obj_set_width(ui_DateText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_DateText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_DateText, 5);
    lv_obj_set_y(ui_DateText, 100);
    lv_label_set_text(ui_DateText, "--/--/--");
    lv_obj_set_style_text_color(ui_DateText, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_DateText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_DateText, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HourText = lv_label_create(ui_Main);
    lv_obj_set_width(ui_HourText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HourText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_HourText, -5);
    lv_obj_set_y(ui_HourText, 100);
    lv_obj_set_align(ui_HourText, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_HourText, "--:--");
    lv_obj_set_style_text_color(ui_HourText, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_HourText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_HourText, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TemperaturesHS = lv_label_create(ui_Main);
    lv_obj_set_width(ui_TemperaturesHS, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TemperaturesHS, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TemperaturesHS, 0);
    lv_obj_set_y(ui_TemperaturesHS, 130);
    lv_obj_set_align(ui_TemperaturesHS, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_TemperaturesHS, "____________________________________");

    ui_TemperaturesLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_TemperaturesLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TemperaturesLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TemperaturesLabel, 0);
    lv_obj_set_y(ui_TemperaturesLabel, 150);
    lv_obj_set_align(ui_TemperaturesLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_TemperaturesLabel, "Temperatures");
    lv_obj_set_style_text_font(ui_TemperaturesLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_InTempLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_InTempLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_InTempLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_InTempLabel, 5);
    lv_obj_set_y(ui_InTempLabel, 165);
    lv_label_set_text(ui_InTempLabel, "In");
    lv_obj_set_style_text_color(ui_InTempLabel, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_InTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_InTempLabel, &lv_font_montserrat_34, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_InTempText = lv_label_create(ui_Main);
    lv_obj_set_width(ui_InTempText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_InTempText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_InTempText, -5);
    lv_obj_set_y(ui_InTempText, 165);
    lv_obj_set_align(ui_InTempText, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_InTempText, "+ --.- °C");
    lv_obj_set_style_text_color(ui_InTempText, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_InTempText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_InTempText, &lv_font_montserrat_34, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_OutTempLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_OutTempLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_OutTempLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_OutTempLabel, 5);
    lv_obj_set_y(ui_OutTempLabel, 200);
    lv_label_set_text(ui_OutTempLabel, "Out");
    lv_obj_set_style_text_color(ui_OutTempLabel, lv_color_hex(0x00FFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_OutTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_OutTempLabel, &lv_font_montserrat_34, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_OutTempText = lv_label_create(ui_Main);
    lv_obj_set_width(ui_OutTempText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_OutTempText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_OutTempText, -5);
    lv_obj_set_y(ui_OutTempText, 200);
    lv_obj_set_align(ui_OutTempText, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_OutTempText, "+ --.- °C");
    lv_obj_set_style_text_color(ui_OutTempText, lv_color_hex(0x00FFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_OutTempText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_OutTempText, &lv_font_montserrat_34, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PressureLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_PressureLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PressureLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_PressureLabel, 5);
    lv_obj_set_y(ui_PressureLabel, 240);
    lv_label_set_text(ui_PressureLabel, "Pressure");
    lv_obj_set_style_text_color(ui_PressureLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_PressureLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_PressureLabel, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PressureText = lv_label_create(ui_Main);
    lv_obj_set_width(ui_PressureText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PressureText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_PressureText, -5);
    lv_obj_set_y(ui_PressureText, 240);
    lv_obj_set_align(ui_PressureText, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_PressureText, "----.- hPa");
    lv_obj_set_style_text_color(ui_PressureText, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_PressureText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_PressureText, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_PressureText, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_PressureText, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_RunTimeHS = lv_label_create(ui_Main);
    lv_obj_set_width(ui_RunTimeHS, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_RunTimeHS, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_RunTimeHS, 0);
    lv_obj_set_y(ui_RunTimeHS, 250);
    lv_obj_set_align(ui_RunTimeHS, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_RunTimeHS, "____________________________________");

    ui_RuntimeLabel = lv_label_create(ui_Main);
    lv_obj_set_width(ui_RuntimeLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_RuntimeLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_RuntimeLabel, 0);
    lv_obj_set_y(ui_RuntimeLabel, 270);
    lv_obj_set_align(ui_RuntimeLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_RuntimeLabel, "Run Time");
    lv_obj_set_style_text_font(ui_RuntimeLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_RuntimeText = lv_label_create(ui_Main);
    lv_obj_set_width(ui_RuntimeText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_RuntimeText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_RuntimeText, 0);
    lv_obj_set_y(ui_RuntimeText, 280);
    lv_obj_set_align(ui_RuntimeText, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_RuntimeText, "--h --m");
    lv_obj_set_style_text_color(ui_RuntimeText, lv_color_hex(0xFFA500), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_RuntimeText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_RuntimeText, &lv_font_montserrat_34, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_SettingBtn, ui_event_SettingBtn, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_PressureHumidityBtn, ui_event_PressureHumidityBtn, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ResetRuntimeBtn, ui_event_ResetRuntimeBtn, LV_EVENT_ALL, NULL);

}
