// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: TermometroAereo

#include "../ui.h"

void ui_SplashScreen_screen_init(void)
{
    ui_SplashScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_SplashScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SplashScreen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SplashScreen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_AliDiClasseLogo = lv_img_create(ui_SplashScreen);
    lv_img_set_src(ui_AliDiClasseLogo, &ui_img_adc_logo_png);
    lv_obj_set_width(ui_AliDiClasseLogo, LV_SIZE_CONTENT);   /// 200
    lv_obj_set_height(ui_AliDiClasseLogo, LV_SIZE_CONTENT);    /// 106
    lv_obj_set_x(ui_AliDiClasseLogo, 0);
    lv_obj_set_y(ui_AliDiClasseLogo, 15);
    lv_obj_set_align(ui_AliDiClasseLogo, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_AliDiClasseLogo, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_AliDiClasseLogo, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_InfoLabel = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_InfoLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_InfoLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_InfoLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_InfoLabel, "Digital\nClock & Thermometer");
    lv_obj_set_style_text_color(ui_InfoLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_InfoLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_InfoLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_InfoLabel, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_VersionLabel = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_VersionLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_VersionLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_VersionLabel, 0);
    lv_obj_set_y(ui_VersionLabel, 55);
    lv_obj_set_align(ui_VersionLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_VersionLabel, "Version 1.0\n27 June 2023");
    lv_obj_set_style_text_align(ui_VersionLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_VersionLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_CreditsLabel = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_CreditsLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_CreditsLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_CreditsLabel, 0);
    lv_obj_set_y(ui_CreditsLabel, 105);
    lv_obj_set_align(ui_CreditsLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_CreditsLabel, "Proudly made by");
    lv_obj_set_style_text_align(ui_CreditsLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_CreditsLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BernaLogo = lv_img_create(ui_SplashScreen);
    lv_img_set_src(ui_BernaLogo, &ui_img_berna_logo_png);
    lv_obj_set_width(ui_BernaLogo, LV_SIZE_CONTENT);   /// 125
    lv_obj_set_height(ui_BernaLogo, LV_SIZE_CONTENT);    /// 23
    lv_obj_set_x(ui_BernaLogo, 0);
    lv_obj_set_y(ui_BernaLogo, -15);
    lv_obj_set_align(ui_BernaLogo, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_BernaLogo, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_BernaLogo, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

}