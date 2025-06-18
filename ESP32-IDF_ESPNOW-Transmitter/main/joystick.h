#ifndef JOYSTICK_H
#define JOYSTICK_H

esp_err_t joystick_adc_init(void);
void joystick_show_raw_xy();
void get_joystick_xy(int *x_axis, int *y_axis);
void sendData (void);
void deletePeer (void);


#endif