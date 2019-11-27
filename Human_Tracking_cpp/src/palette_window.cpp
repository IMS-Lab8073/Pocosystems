#include "Human_Tracking.h"
#include "windows.h"
#include <tchar.h>
#include <time.h>

#define M_PI  3.14159265358979323846
#define CENTER_X 250
#define CENTER_Y 300

HINSTANCE Inst;
HWND hWnd;
MSG msg;
WNDCLASSEX wc;
HDC hdc;
HDC memDC,hBlocksBmpDC;
double pixel2dist=18;
int m_NUM;
long range[1100];
Human_Tracking::Humans m_qHuman;
Human_Tracking::Data m_data; 

//Particle_Tracking::particle_type draw_particles[max_particles];
//Particle_Tracking::object draw_object[10];
double draw_scale=1.0;

int draw_window(HWND hWnd)
{
	hdc=GetDC(hWnd);
	static HBITMAP  hMemBmp,hOldMemBmp;
	HPEN pen_red = CreatePen( PS_SOLID, 1, RGB(255,0,0) );
	HPEN pen_blue = CreatePen( PS_SOLID, 1, RGB(0,0,255) );
	HPEN pen_green = CreatePen( PS_SOLID, 1, RGB(0,255,0) );
	HPEN pen_yellow = CreatePen( PS_SOLID, 1, RGB(255,255,0) );
	HPEN pen_black = CreatePen( PS_SOLID, 1, RGB(0,0,0) );
	HPEN pen_white_1 = CreatePen( PS_SOLID, 1, RGB(255,255,255) );
	HPEN pen_white_2 = CreatePen( PS_SOLID, 2, RGB(255,255,255) );
	HPEN pen_white_3 = CreatePen( PS_SOLID, 3, RGB(255,255,255) );
	HPEN pen_gray = CreatePen( PS_SOLID, 1, RGB(125,125,125) );
	HPEN pen_blue2 = CreatePen(PS_SOLID, 1, RGB(74, 115, 255));	
	HPEN pen_blue3 = CreatePen(PS_SOLID, 5, RGB(0,0,255));
	HPEN pen_green2 = CreatePen( PS_SOLID, 5, RGB(0,255,0) );
	HPEN pen_yellow2 = CreatePen( PS_SOLID, 5, RGB(255,255,0) );
	HPEN pen_black2 = CreatePen( PS_SOLID, 5, RGB(0,0,0) );
	HPEN pen_red2 = CreatePen( PS_SOLID, 4, RGB(255,0,0) );
	HPEN pen_gray2 = CreatePen( PS_SOLID, 5, RGB(125,125,125) );
	/*
		HBRUSH br_blue3 = CreateHatchBrush(HS_API_MAX,  RGB(255,0,0));
	HBRUSH br_green2 = CreateHatchBrush( HS_API_MAX,  RGB(0,255,0) );
	HBRUSH br_yellow2 = CreateHatchBrush( HS_API_MAX,  RGB(255,255,0) );
	HBRUSH br_black2 = CreateHatchBrush( HS_API_MAX,  RGB(0,0,0) );
	HBRUSH br_red2 = CreateHatchBrush( HS_API_MAX,  RGB(255,0,0) );
	HBRUSH br_gray2 = CreateHatchBrush( HS_API_MAX, RGB(125,125,125) );*/

	// メモリデバイスコンテキストを作成する
	hMemBmp = CreateCompatibleBitmap( hdc, 500, 500 );
	memDC = CreateCompatibleDC( hdc );
	hOldMemBmp = (HBITMAP)SelectObject( memDC, hMemBmp );
	
	//軸の描画
	SelectObject( memDC, pen_white_3 );
	MoveToEx(memDC , CENTER_X , 0 , NULL);
	LineTo(memDC , CENTER_X , 500);
	MoveToEx(memDC ,0 , CENTER_Y , NULL);
	LineTo(memDC , 500 , CENTER_Y);
	
/*	//目盛り描画(50pixel区切り)
	SelectObject( memDC, pen_white_1 );
	for(int count=0;count<=10/draw_scale;count++)
	{
		MoveToEx(memDC ,count*50*draw_scale ,0 , NULL);
		LineTo(memDC ,count*50*draw_scale ,500);
		MoveToEx(memDC ,0 ,count*50*draw_scale , NULL);
		LineTo(memDC ,500 ,count*50*draw_scale);
	}*/




	//測域の外周を描画する処理
	MoveToEx(memDC ,CENTER_X ,CENTER_Y , NULL);
	SelectObject( memDC, pen_blue2 );
	for(int count=m_data.LRF_start;count<m_data.LRF_end;count++)
	{

		//if(m_data.mem_data_x[count]/m_data.mem_data_x2[count]<0.9){
			double direction=(count-m_data.LRF_mid)*m_data.LRF_theta/360*2*M_PI/m_data.LRF_steps; 
			double point_x=m_data.mem_data_x2[count]*sin(direction)/(pixel2dist*draw_scale);
			double point_y=m_data.mem_data_x2[count]*cos(direction)/(pixel2dist*draw_scale);		
			LineTo(memDC ,CENTER_X+point_x ,CENTER_Y-point_y);	
		//}
	}

	//計測データの描画
	//SelectObject( memDC, pen_blue2 );
	for(int count=m_data.LRF_start;count<m_data.LRF_end;count++)
	{
		if(m_data.m_different_search[count]==1){
			SelectObject( memDC, pen_green );
		/*if(m_data.parson_divide[1][count]==1)SelectObject( memDC, pen_green );
		if(m_data.parson_divide[2][count]==1)SelectObject( memDC, pen_yellow );
		if(m_data.parson_divide[3][count]==1)SelectObject( memDC, pen_gray  );
		if(m_data.parson_divide[4][count]==1)SelectObject( memDC, pen_white_1 );
		if(m_data.parson_divide[5][count]==1)SelectObject( memDC, pen_white_1);
		if(m_data.parson_divide[6][count]==1)SelectObject( memDC, pen_white_1 );*/
	}
		else SelectObject( memDC, pen_blue2 );
			double direction=(count-m_data.LRF_mid)*m_data.LRF_theta/360*2*M_PI/m_data.LRF_steps; 
			double point_x=m_data.mem_data_x[count]*sin(direction)/(pixel2dist*draw_scale);
			double point_y=m_data.mem_data_x[count]*cos(direction)/(pixel2dist*draw_scale);
			MoveToEx(memDC ,CENTER_X ,CENTER_Y , NULL);
			LineTo(memDC ,CENTER_X+point_x ,CENTER_Y-point_y);
			
	}
	
	





	//検出データの描画
	for(int i=1;i<=m_NUM;i++)
	{
		for(int i2=0;i2<max_particles;i2++)
		{
		SetPixel(memDC,CENTER_X+m_qHuman.human[i].yp[i2]/(pixel2dist*draw_scale),CENTER_Y-m_qHuman.human[i].xp[i2]/(pixel2dist*draw_scale),RGB(255,255,0));
		}
		//SelectObject( memDC, pen_blue );
		if(m_qHuman.human[i].y2[1]<4000 && m_qHuman.human[i].y2[1]!=0){
		SelectObject( memDC, pen_red2 );
		/*if(i==1)SelectObject( memDC, pen_black2 );
		else if(i==2)SelectObject( memDC, pen_blue3 );
		else if(i==3)SelectObject( memDC, pen_green2 );
		else if(i==4)SelectObject( memDC, pen_yellow2 );
		else if(i==5)SelectObject( memDC, pen_gray2 );
		else if(i==6)SelectObject( memDC, pen_red2 );
		else SelectObject( memDC, pen_black2 );*/
		double point_x=m_qHuman.human[i].y2[1];
		double point_y=m_qHuman.human[i].x2[1];		
		int draw_x=CENTER_X+point_x/(pixel2dist*draw_scale);
		int draw_y=CENTER_Y-point_y/(pixel2dist*draw_scale);
		Ellipse(memDC,draw_x-20,draw_y-20,draw_x+20,draw_y+20);
		if(i==1)TextOut( memDC, draw_x-8, draw_y-8, "01",2 );
		else if(i==2)TextOut( memDC, draw_x-8, draw_y-8, "02",2 );
		else if(i==3)TextOut( memDC, draw_x-8, draw_y-8, "03",2 );
		else if(i==4)TextOut( memDC, draw_x-8, draw_y-8, "04",2 );
		else if(i==5)TextOut( memDC, draw_x-8, draw_y-8, "05",2 );
		else if(i==6)TextOut( memDC, draw_x-8, draw_y-8, "06",2 );
		else if(i==7)TextOut( memDC, draw_x-8, draw_y-8, "07",2 );
		else if(i==8)TextOut( memDC, draw_x-8, draw_y-8, "08",2 );
		else if(i==9)TextOut( memDC, draw_x-8, draw_y-8, "09",2 );
		else if(i==10)TextOut( memDC, draw_x-8, draw_y-8, "10",2 );
		else {}
		}
	}

	//std::cout << "Data is " << m_qHuman.human[1].y2[1]<< std::endl;


/*	//パーティクルの描画
	for(int j=0;j<max_particles;j++)
	{
		int color=((double)200*(double)draw_particles[j].weight);
		HPEN hPen_create = CreatePen( PS_SOLID, 2, RGB(color,10,10) ); // 赤の論理ペンを作成する
		SelectObject( memDC, hPen_create );
		Ellipse( memDC,
		-draw_particles[j].y-3+250,
		-draw_particles[j].x-3+300,
		-draw_particles[j].y+3+250,
		-draw_particles[j].x+3+300);
		DeleteObject( hPen_create );
	}

	//検出データの描画
	for(int count=0;count<10;count++)
	{
		SelectObject( memDC, pen_blue );
		double point_x=draw_object[count].x;
		double point_y=draw_object[count].y;
		
		int draw_x=CENTER_X+point_x/pixel2dist*draw_scale;
		int draw_y=CENTER_Y-point_y/pixel2dist*draw_scale;
		Ellipse(memDC,draw_x-5,draw_y-5,draw_x+5,draw_y+5);
	}*/

	BitBlt(hdc , 0 , 0 , 500 , 500 , memDC , 0 , 0 , SRCCOPY);

	//DeleteObject( hBlocksBmp );  // ロードしたビットマップを削除する
	DeleteObject( hMemBmp );
	DeleteObject( hOldMemBmp );
	DeleteObject( pen_red );
	DeleteObject( pen_blue );
	DeleteObject( pen_green );
	DeleteObject( pen_yellow );
	DeleteObject( pen_black );
	DeleteObject( pen_gray );
	DeleteObject( pen_white_1 );
	DeleteObject( pen_white_2 );
	DeleteObject( pen_white_3 );
	DeleteObject( pen_red2 );
	DeleteObject( pen_blue2 );
	DeleteObject( pen_blue3 );
	DeleteObject( pen_green2 );
	DeleteObject( pen_yellow2 );
	DeleteObject( pen_black2 );
	DeleteObject( pen_gray2 );
	ReleaseDC(hWnd , hdc);
	DeleteDC(memDC);
	return 0;

}

// ウィンドウプロシージャ
LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wp, LPARAM lp)
{
	switch( msg )
	{
	case WM_DESTROY:  // ウィンドウを破棄するとき
		PostQuitMessage( 0 );
		return 0;
	case WM_PAINT:
		draw_window(hWnd);
		return 0;
	}

	// 他のメッセージは、デフォルトの処理を行う
	return DefWindowProc( hWnd, msg, wp, lp );
}

int WINAPI window_create(HINSTANCE hInst,HWND hWnd)
{
	// ウィンドウクラスの情報を設定
	wc.cbSize = sizeof(wc);               // 構造体サイズ
	wc.style = CS_HREDRAW | CS_VREDRAW;   // スタイル
	wc.lpfnWndProc = WndProc;             // ウィンドウプロシージャ
	wc.cbClsExtra = 0;                    // 拡張情報１
	wc.cbWndExtra = 0;                    // 拡張情報２
	wc.hInstance = hInst;                 // インスタンスハンドル
	wc.hIcon = (HICON)LoadImage(          // アイコン
		NULL, MAKEINTRESOURCE(IDI_APPLICATION), IMAGE_ICON,
		0, 0, LR_DEFAULTSIZE | LR_SHARED
	);
	wc.hIconSm = wc.hIcon;                // 子アイコン
	wc.hCursor = (HCURSOR)LoadImage(      // マウスカーソル
		NULL, MAKEINTRESOURCE(IDC_ARROW), IMAGE_CURSOR,
		0, 0, LR_DEFAULTSIZE | LR_SHARED
	);
	wc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH); // ウィンドウ背景
	wc.lpszMenuName = NULL;                     // メニュー名
	wc.lpszClassName = _T("Default Class Name");// ウィンドウクラス名
	
	// ウィンドウクラスを登録する
	if( RegisterClassEx( &wc ) == 0 ){ return 1; }

	// ウィンドウを作成する
	hWnd = CreateWindow(
		wc.lpszClassName,      // ウィンドウクラス名
		_T("Human_Tracking"),  // タイトルバーに表示する文字列
		WS_OVERLAPPEDWINDOW,   // ウィンドウの種類
		CW_USEDEFAULT,         // ウィンドウを表示する位置（X座標）
		CW_USEDEFAULT,         // ウィンドウを表示する位置（Y座標）
		500,         // ウィンドウの幅
		500,         // ウィンドウの高さ
		NULL,                  // 親ウィンドウのウィンドウハンドル
		NULL,                  // メニューハンドル
		hInst,                 // インスタンスハンドル
		NULL                   // その他の作成データ
	);
	if( hWnd == NULL ){ return 1; }

	// ウィンドウを表示する
	ShowWindow( hWnd, SW_SHOW );
	UpdateWindow( hWnd );
	return 0;
}

void window_init()
{
	Inst=GetModuleHandle(NULL);
	window_create(Inst,hWnd);
	BOOL ret2 = GetMessage( &msg, NULL, 0, 0 );  // メッセージを取得する
	if( ret2 == 0 || ret2 == -1 )
	{
		// アプリケーションを終了させるメッセージが来ていたら、
		// あるいは GetMessage() が失敗したら( -1 が返されたら ）、ループを抜ける
	}
	else
	{
		// メッセージを処理する
		TranslateMessage( &msg );
		DispatchMessage( &msg );
	}
}

void window_reload()
{
	BOOL ret = GetMessage( &msg, NULL, 0, 0 );  // メッセージを取得する
	if( ret == 0 || ret == -1 )
	{
		// アプリケーションを終了させるメッセージが来ていたら、
		// あるいは GetMessage() が失敗したら( -1 が返されたら ）、ループを抜ける
	}
	else
	{
		// メッセージを処理する
		TranslateMessage( &msg );
		DispatchMessage( &msg );
	}
	UpdateWindow( hWnd );
}

void copy_data(RTC::RangeData original_range)
{
	for(int count=m_data.LRF_start;count<m_data.LRF_end;count++)
	{
		range[count]=(long)original_range.ranges[count];
	}
}

void copy_data2(Human_Tracking::Data *m_DATA2,int m_NUM_MAX)
{   m_NUM=m_NUM_MAX;
	m_data.LRF_start=m_DATA2->LRF_start;
	m_data.LRF_end=m_DATA2->LRF_end;
	m_data.LRF_mid=m_DATA2->LRF_mid;
    m_data.LRF_theta=m_DATA2->LRF_theta;
	m_data.LRF_steps=m_DATA2->LRF_steps;

	for(int count=m_data.LRF_start;count<m_data.LRF_end;count++)
	{
		m_data.mem_data_x[count]=m_DATA2->mem_data_x[count];
		m_data.mem_data_x2[count]=m_DATA2->mem_data_x2[count];
		m_data.m_different_search[count]=m_DATA2->m_different_search[count];
		
	}
	for(int i=1;i<=m_NUM;i++)
	{
		for(int count=m_data.LRF_start;count<m_data.LRF_end;count++)
		{
         m_data.parson_divide[i][count]=m_DATA2->parson_divide[i][count];
		}
	
	}
}

void copy_data3(Human_Tracking::Humans *m_DATA3)
{
	
	for(int i=1;i<=m_NUM;i++)
	{
		m_qHuman.human[i].x2[1]=m_DATA3->human[i].x2[1];
		m_qHuman.human[i].y2[1]=m_DATA3->human[i].y2[1];
		m_qHuman.human[i].x3[1]=m_DATA3->human[i].x3[1];
		m_qHuman.human[i].y3[1]=m_DATA3->human[i].y3[1];
	for(int i2=0;i2<max_particles;i2++)
	{
		m_qHuman.human[i].xp[i2]=m_DATA3->human[i].xp[i2];
		m_qHuman.human[i].yp[i2]=m_DATA3->human[i].yp[i2];
	}
	}
}
/*void copy_object_data(Human_Tracking::Humans obj[])
{
	for(int count=0;count<10;count++)
	{
		m_qHuman[count].x=obj[count].x;
		m_qHuman[count].y=obj[count].y;
	}
}*/