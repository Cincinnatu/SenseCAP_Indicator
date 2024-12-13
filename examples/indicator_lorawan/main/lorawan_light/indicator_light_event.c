#include "ui.h"
#include "ui_events.h"

#include "view_data.h"

char buffer[6] = {"Hello"};
extern int PWM_SET;

void light_ctrl(bool light_st)
{
    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_LORAWAN_LIGHT_ST, (void *)&light_st, sizeof(bool), portMAX_DELAY);
}

void lora_light_on(lv_event_t *e)
{
    //light_ctrl(true);
    //ESP_LOGI("on");
   // sprintf(buffer, "XXX");
   PWM_SET = 1 ;
   //ESP_LOGI("MEM", "%s" "%d", buffer,PWM_SET);

}
void lora_light_off(lv_event_t *e)
{
    PWM_SET = 0;
    //ESP_LOGI("MEM", "%s" "%d", buffer,PWM_SET);
    //light_ctrl(false);
    //ESP_LOGI("off");
}