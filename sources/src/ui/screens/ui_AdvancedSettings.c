// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: TermometroAereo

#include "../ui.h"

void ui_AdvancedSettings_screen_init(void)
{
    ui_AdvancedSettings = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_AdvancedSettings, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_AdvancedSettings, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_AdvancedSettings, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_AdvancedSettingsLabel = lv_label_create(ui_AdvancedSettings);
    lv_obj_set_width(ui_AdvancedSettingsLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_AdvancedSettingsLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AdvancedSettingsLabel, 0);
    lv_obj_set_y(ui_AdvancedSettingsLabel, 10);
    lv_obj_set_align(ui_AdvancedSettingsLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_AdvancedSettingsLabel, "Advanced Settings");

    ui_AdjustInsideTempLabel = lv_label_create(ui_AdvancedSettings);
    lv_obj_set_width(ui_AdjustInsideTempLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_AdjustInsideTempLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AdjustInsideTempLabel, 5);
    lv_obj_set_y(ui_AdjustInsideTempLabel, 40);
    lv_label_set_text(ui_AdjustInsideTempLabel, "Adjust inside\ntemperature");

    ui_AdjustInsideTempDropdown = lv_dropdown_create(ui_AdvancedSettings);
    lv_dropdown_set_options(ui_AdjustInsideTempDropdown,
                            "-5.0\n-4.9\n-4.8\n-4.7\n-4.6\n-4.5\n-4.4\n-4.3\n-4.2\n-4.1\n-4.0\n-3.9\n-3.8\n-3.7\n-3.6\n-3.5\n-3.4\n-3.3\n-3.2\n-3.1\n-3.0\n-2.9\n-2.8\n-2.7\n-2.6\n-2.5\n-2.4\n-2.3\n-2.2\n-2.1\n-2.0\n-1.9\n-1.8\n-1.7\n-1.6\n-1.5\n-1.4\n-1.3\n-1.2\n-1.1\n-1.0\n-0.9\n-0.8\n-0.7\n-0.6\n-0.5\n-0.4\n-0.3\n-0.2\n-0.1\n0\n0.1\n0.2\n0.3\n0.4\n0.5\n0.6\n0.7\n0.8\n0.9\n1.0\n1.1\n1.2\n1.3\n1.4\n1.5\n1.6\n1.7\n1.8\n1.9\n2.0\n2.1\n2.2\n2.3\n2.4\n2.5\n2.6\n2.7\n2.8\n2.9\n3.0\n3.1\n3.2\n3.3\n3.4\n3.5\n3.6\n3.7\n3.8\n3.9\n4.0\n4.1\n4.2\n4.3\n4.4\n4.5\n4.6\n4.7\n4.8\n4.9\n5.0");
    lv_obj_set_width(ui_AdjustInsideTempDropdown, 60);
    lv_obj_set_height(ui_AdjustInsideTempDropdown, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AdjustInsideTempDropdown, -5);
    lv_obj_set_y(ui_AdjustInsideTempDropdown, 39);
    lv_obj_set_align(ui_AdjustInsideTempDropdown, LV_ALIGN_TOP_RIGHT);
    lv_obj_add_flag(ui_AdjustInsideTempDropdown, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_AdjustOutsideTempLabel = lv_label_create(ui_AdvancedSettings);
    lv_obj_set_width(ui_AdjustOutsideTempLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_AdjustOutsideTempLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AdjustOutsideTempLabel, 5);
    lv_obj_set_y(ui_AdjustOutsideTempLabel, 80);
    lv_label_set_text(ui_AdjustOutsideTempLabel, "Adjust outside\ntemperature");

    ui_AdjustOutsideTempDropdown = lv_dropdown_create(ui_AdvancedSettings);
    lv_dropdown_set_options(ui_AdjustOutsideTempDropdown,
                            "-5.0\n-4.9\n-4.8\n-4.7\n-4.6\n-4.5\n-4.4\n-4.3\n-4.2\n-4.1\n-4.0\n-3.9\n-3.8\n-3.7\n-3.6\n-3.5\n-3.4\n-3.3\n-3.2\n-3.1\n-3.0\n-2.9\n-2.8\n-2.7\n-2.6\n-2.5\n-2.4\n-2.3\n-2.2\n-2.1\n-2.0\n-1.9\n-1.8\n-1.7\n-1.6\n-1.5\n-1.4\n-1.3\n-1.2\n-1.1\n-1.0\n-0.9\n-0.8\n-0.7\n-0.6\n-0.5\n-0.4\n-0.3\n-0.2\n-0.1\n0\n0.1\n0.2\n0.3\n0.4\n0.5\n0.6\n0.7\n0.8\n0.9\n1.0\n1.1\n1.2\n1.3\n1.4\n1.5\n1.6\n1.7\n1.8\n1.9\n2.0\n2.1\n2.2\n2.3\n2.4\n2.5\n2.6\n2.7\n2.8\n2.9\n3.0\n3.1\n3.2\n3.3\n3.4\n3.5\n3.6\n3.7\n3.8\n3.9\n4.0\n4.1\n4.2\n4.3\n4.4\n4.5\n4.6\n4.7\n4.8\n4.9\n5.0");
    lv_obj_set_width(ui_AdjustOutsideTempDropdown, 60);
    lv_obj_set_height(ui_AdjustOutsideTempDropdown, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AdjustOutsideTempDropdown, -5);
    lv_obj_set_y(ui_AdjustOutsideTempDropdown, 79);
    lv_obj_set_align(ui_AdjustOutsideTempDropdown, LV_ALIGN_TOP_RIGHT);
    lv_obj_add_flag(ui_AdjustOutsideTempDropdown, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_BackSettingsBtn = lv_btn_create(ui_AdvancedSettings);
    lv_obj_set_width(ui_BackSettingsBtn, 110);
    lv_obj_set_height(ui_BackSettingsBtn, 50);
    lv_obj_set_x(ui_BackSettingsBtn, 0);
    lv_obj_set_y(ui_BackSettingsBtn, -10);
    lv_obj_set_align(ui_BackSettingsBtn, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_BackSettingsBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BackSettingsBtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BackSettingsBtn, lv_color_hex(0xFA8500), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BackSettingsBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BackSettingsBtnLabel = lv_label_create(ui_BackSettingsBtn);
    lv_obj_set_width(ui_BackSettingsBtnLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_BackSettingsBtnLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_BackSettingsBtnLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_BackSettingsBtnLabel, "Back");

    lv_obj_add_event_cb(ui_AdjustInsideTempDropdown, ui_event_AdjustInsideTempDropdown, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_AdjustOutsideTempDropdown, ui_event_AdjustOutsideTempDropdown, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BackSettingsBtn, ui_event_BackSettingsBtn, LV_EVENT_ALL, NULL);

}
