#include "Human_Tracking.h"

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wp, LPARAM lp);
int WINAPI window_create(HINSTANCE hInst,HWND hWnd);
int draw_window();
void window_init();
void window_reload();
void copy_data(RTC::RangeData original_range);
void copy_data2(Human_Tracking::Data *m_DATA2,int m_NUM_MAX);
void copy_data3(Human_Tracking::Humans *m_DATA3);
//void copy_object_data(Particle_Tracking::object obj[]);

