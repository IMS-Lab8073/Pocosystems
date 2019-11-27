// -*- C++ -*-
/*!
 * @file  Human_Tracking.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * @author Daiki Nomiyama, Shinichi Ishida, Shogo Namatame, Nobuto Matsuhira / SHIBAURA INSTITUTE OF TECHNOLOGY /  3-7-5 TOYOSU KOUTO-KU TOKYO 135-8548
 *�@�@�@�@ Email:md14067@shibaura-it.ac.jp
 *         Tell : 03-5859-8073
 * @author �쌩�R���@�ŉY�H�Ƒ�w�@135-8548 �����s�]����L�F3-7-5
 * 
 *
 *
 * ���C�Z���X�F
 * �ȉ���MIT Licence��K�p���܂��D
 * ////////////////////////////////////////////////////////////////////////////////////////////////////

	Copyright (c) 2003-2009, MIST Project, Intelligent Media Integration COE, Nagoya University
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

		1. Redistributions of source code must retain the above copyright notice,
		   this list of conditions and the following disclaimer. 

		2. Redistributions in binary form must reproduce the above copyright notice,
		   this list of conditions and the following disclaimer in the documentation
		   and/or other materials provided with the distribution.

		3. Neither the name of the Nagoya University nor the names of its contributors
		   may be used to endorse or promote products derived from this software
		   without specific prior written permission. 

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
	FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
	IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
	THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	//////////////////////////////////////////////////////////////////////////////////////////////////////
 * http://mist.suenaga.cse.nagoya-u.ac.jp/trac/wiki/License (Last accessed 2014/09/24)
 *
 *
 * $Id$
 */


#include "Human_Tracking.h"
#include "palette_window.h"
#include <windows.h>
#include <tchar.h>

#define M_PI            3.14159265358979323846
#define max_particles   100
#define cut_rate        0.6
#define NUM_Hmax         11

// Module specification
// <rtc-template block="module_spec">
static const char* human_tracking_spec[] =
  {
    "implementation_id", "Human_Tracking",
    "type_name",         "Human_Tracking",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "Nomiyama",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.Human_NUM", "6",
    "conf.default.Environmental_maxdata", "4000",
    "conf.default.LRFdata_min", "44",
    "conf.default.LRFdata_max", "725",
	"conf.default.LRFdata_mid", "384",
	"conf.default.LRF_Total_steps", "1024",
	"conf.default.LRFdata_theta", "360",
    "conf.default.Human_width", "200",
    "conf.default.Count_mode", "IN,OUT",
	"conf.default.Human_setdistance", "2800",
	"conf.default.Human_TrackingData", "2500",
	"conf.default.Count_area_distance", "2500",
    // Widget
    "conf.__widget__.Human_NUM", "text",
    "conf.__widget__.Environmental_maxdata", "text",
    "conf.__widget__.LRFdata_min", "text",
    "conf.__widget__.LRFdata_max", "text",
	"conf.__widget__.LRFdata_mid", "text",
	"conf.__widget__.LRFdata_theta", "text",
	"conf.__widget__.LRF_Total_steps", "text",
    "conf.__widget__.Human_width", "text",
    "conf.__widget__.Count_mode", "text",
	"conf.__widget__.Human_setdistance", "text",
	"conf.__widget__.Human_TrackingData", "text",
	"conf.__widget__.Count_area_distance", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Human_Tracking::Human_Tracking(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_range_dataIn("range_data", m_range_data),
	m_humanXOut("human_dataX", m_humanX),
	m_humanYOut("human_dataY", m_humanY),
	m_humanVOut("human_dataV", m_humanV),
	m_humanTOut("human_dataT", m_humanT),
	m_hcountOut("count_data", m_hcount)
	//m_pointOut("Point_data",m_point)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Human_Tracking::~Human_Tracking()
{
}



RTC::ReturnCode_t Human_Tracking::onInitialize()
{ 
  addInPort("range_data", m_range_dataIn);
  addOutPort("human_dataX", m_humanXOut);
  addOutPort("human_dataY", m_humanYOut);
  addOutPort("human_dataV", m_humanVOut);
  addOutPort("human_dataT", m_humanTOut);
  addOutPort("count_data", m_hcountOut);
  //addOutPort("Point_data", m_pointOut);
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("Human_NUM", m_NUM_max, "6");
  bindParameter("Environmental_maxdata", m_environment, "4000");
  bindParameter("LRFdata_min", m_LRFdata_min, "44");
  bindParameter("LRFdata_max", m_LRFdata_max, "725");
  bindParameter("LRFdata_mid", m_LRFdata_mid, "384");
  bindParameter("LRFdata_theta", m_LRFdata_theta, "360");
  bindParameter("LRF_Total_steps", m_LRF_steps, "1024");
  bindParameter("Human_width", m_human_width, "200");
  bindParameter("Count_mode", m_Count_mode, "IN,OUT");
  bindParameter("Count_area_distance", m_Count_area, "2500");
  bindParameter("Human_setdistance", m_Human_set, "2800");
  bindParameter("Human_TrackingData", m_Human_TrackingData, "2500");
  
  // </rtc-template>
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Human_Tracking::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Human_Tracking::onActivated(RTC::UniqueId ec_id)
{
	window_init();       //�`�ʃE�B���h�E����
     Human_Initialize();  //������
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Human_Tracking::onDeactivated(RTC::UniqueId ec_id)
{
	delete [] m_qHuman;  //�e�l�̕��s���i�\���́j�f�[�^���
	delete [] m_qHuman2; //�e�l�̕��s���i�\���́j�f�[�^���
	delete [] m_data;    //���f�[�^�y�ѐ��f�[�^���
	
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Human_Tracking::onExecute(RTC::UniqueId ec_id)
{
	int detectflag = 0;
	int k = 0;
	int kappa = 0;
	if(m_range_dataIn.isNew())
	{	
		LRFData_In();						//�V�����f�[�^���擾
		strong_pass_filter(10,6);			//���x����߂���f�[�^�̗L���t���O��܂�i���ݎ��j{�O��̃X�L�����͈�,�L���f�[�^��}�ȉ��̃f�[�^���Ώ�
		weak_add_filter(5,5);				//���x�������ꏊ�ɂ���f�[�^�̌�����⊮�i�����߁j
		parson_devid_method(10,4,3,300);	//�T�u�v�����ʂƂ��ē���ꂽ�f�[�^���I�u�W�W�F�F�N�g���Ƃɕ�����
		LRFPocoHuman();						//�}�b�`���O�ɂ�镡���l�ǐ�

		//�\������
		copy_data2(m_data,m_NUM_max);	    //LRF�f�[�^��`�ʊ֐��ɃR�s�[
		copy_data(m_range_data);			//LRF�f�[�^��`�ʊ֐��ɃR�s�[
		copy_data3(m_qHuman);				//�l�f�[�^��`�ʊ֐��ɃR�s�[
		
		
	}
	else{}

	time_t timer;
	struct tm *t_st;
	time(&timer);
	t_st = localtime(&timer);
	char filename[50];

	window_reload();					//�`�ʍX�V
	Data_outport();						//�n�t�s�o�n�q�s�Ƀf�[�^��n��


   return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Human_Tracking::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

//�V�f�[�^�����֐�/���f�[�^�C�y�ъ��f�[�^�d�����֐�
void Human_Tracking::LRFData_In(){

	m_data->LRF_start=m_LRFdata_min;                                           //�`�ʗp�ɒl��ۑ�
	m_data->LRF_end=m_LRFdata_max;
	m_data->LRF_mid=m_LRFdata_mid;
	m_data->LRF_theta=m_LRFdata_theta;
	m_data->LRF_steps=m_LRF_steps;

	m_range_dataIn.read();
		for(int count = m_LRFdata_min;count <= m_LRFdata_max;count++)	        //����ꂽ�f�[�^��ۑ��ϐ��ɏ���������Ă䂭
		{
		m_data->mem_data_x[count]=m_range_data.ranges[count];
		m_data->mem_data_x4[count] = m_range_data.ranges[count];
		//std::cout << m_range_data.ranges[count] << std::endl;
		if(m_data->mem_data_x[count]>m_environment)m_data->mem_data_x[count]=m_environment;
		if(m_data->mem_data_x[count]<0)m_data->mem_data_x[count]=10;
		if(m_data->mem_data_x2[count]==0)m_data->mem_data_x2[count]=m_data->mem_data_x[count];
		if(m_data->mem_data_x2[count]!=0){                                      //�L���f�[�^������ꂽ�ꍇ�i�f�[�^��0�łȂ��Ƃ��j
			if(m_data->mem_data_x[count]>m_data->mem_data_x2[count]){
				m_data->mem_data_x2[count]=m_data->mem_data_x[count];           //���Ƃ̃f�[�^���������̑傫���f�[�^������ꂽ�ꍇ�͏����X�V����
			}
			else{}

			if(m_data->mem_data_x[count]/m_data->mem_data_x2[count]<0.9){		//���f�[�^����9���ȉ��̏ꍇ�͐l������Ɣ��f�i�w�i�����@�j
				if(m_data->mem_data_x[count]>10){		                        //�f�[�^��10cm�ȉ��̏ꍇ�͖����i�m�C�Y�����̂��߁j
					m_data->m_different_search[count]=1;		                //�f�[�^���L���ȏꏊ�̃t���O�𗧂Ă�(1�X�e�b�v���Ƃɍs���D1�F�L���C0:����)
				}
				else{
				m_data->m_different_search[count]=0;			                 //�����f�[�^�ꍇ�̓t���O��܂�
					}
				}
			else{
			m_data->m_different_search[count]=0;			                     //�����f�[�^�ꍇ�̓t���O��܂�
			}
						
		}
		//��{�ƂȂ�f�[�^��0�̂Ƃ�
		else{
			if(m_data->mem_data_x[count]=m_environment){
				if(m_data->mem_data_x2[count]=m_environment){
					m_data->m_different_search[count]=0;	
				}
			}

		}
		if(m_data->mem_data_x2[count]!=0 && m_data->mem_data_x[count]==0)m_data->mem_data_x[count]=m_data->mem_data_x2[count];
	}
}
// ���x�̒Ⴂ���������O����֐�
void Human_Tracking::strong_pass_filter(int scan_range,int filter_limit)
{
	int filter_on=0;
	//���x�̎ア�f�[�^����Ƀt�B���^����
	for(int i1 = m_LRFdata_min;i1 <= m_LRFdata_max;i1++){
		filter_on=0;
		if(m_data->m_different_search[i1]==1){
			for(int filter_count=1;filter_count<scan_range;filter_count++){		//�e�f�[�^�̑O��10�J�E���g������
				if(m_data->m_different_search[i1-filter_count]==1)filter_on++;
				if(m_data->m_different_search[i1+filter_count]==1)filter_on++;
								
			}
			if(filter_on<filter_limit){			//�����̈�ɗL���f�[�^��10�ȉ��ȏꍇ�̓f�[�^�𖳌����i�m�C�Y�̏����j
			m_data->m_different_search[i1]=0;
			move_trace[0][0]-=1;
			move_trace[0][1]-=i1;
			}
		}
	}

}
//���x�̍����ӏ��ɂ��錇���f�[�^��⊮����֐�
void Human_Tracking::weak_add_filter(int scan_range,int filter_limit)
{
	
		double mem_data_x_ave=0;
	//�������ϒl����傫���O���l�̃t���O��܂�
			for(int i1 = m_LRFdata_min;i1 <= m_LRFdata_max;i1++){
		if(m_data->m_different_search[i1]==1){
		for(int filter_count2=1;filter_count2<=scan_range;filter_count2++){		//�e�f�[�^�̑O10�J�E���g������
			mem_data_x_ave+=m_data->mem_data_x[i1];
		}	
			mem_data_x_ave=mem_data_x_ave/scan_range;	
		for(int filter_count=1;filter_count<=scan_range;filter_count++){		
			if(abs(mem_data_x_ave-m_data->mem_data_x[i1+filter_count])>300)m_data->m_different_search[i1+filter_count]==0;
		}
		}
			
		else{}
			}

}
//�����̃I�u�W�F�N�g�𕪊�����֐�
void Human_Tracking::parson_devid_method(int scan_range,int filter_limit,int delsize,int LimitAveDistance)
{
	int parson_X=1;
	//�w�i�����@�ɂ�蓾��ꂽ1�X�e�b�v���Ƃ̗L���f�[�^�̘A�������m�F
	for(int i1 = m_LRFdata_min;i1 <= m_LRFdata_max;i1++){
			if (m_data->m_different_search[i1] == 1) {
				trans_memdata_to_pxpy(i1);
				human_sx[i1] = ppx_d;
				human_sy[i1] = ppy_d;
				trans_memdata_to_pxpy(i1 + 1);
				human_ex[i1 + 1] = ppx_d;
				human_ey[i1 + 1] = ppy_d;
				human_dr[i1] = hypot((human_sx[i1] - human_ex[i1 + 1]), (human_sy[i1] - human_ey[i1 + 1]));

				if ((m_data->m_different_search[i1] - m_data->m_different_search[i1 - 1]) == 1) {
					flag_check_start[parson_X]=i1;
				}
				else if (m_data->m_different_search[i1] - m_data->m_different_search[i1 + 1] == 0) {
					if (human_dr[i1] > 200) {
						flag_check_end[parson_X] = i1;
					}
					else if (human_dr[i1 - 1] > 200 && (m_data->m_different_search[i1] - m_data->m_different_search[i1 - 1] == 0)) {
						flag_check_start[parson_X] = i1;
					}
				}
				else if (m_data->m_different_search[i1] - m_data->m_different_search[i1 + 1] == 1) {
					if (human_dr[i1 - 1] > 200) {
						flag_check_start[parson_X] = 0;
					}
					else {
						flag_check_end[parson_X] = i1;
					}
				}
				else {}

			}
		//������x�A�������L���f�[�^�ɑ΂��ĘA�������L���X�e�b�v�̕����v�Z
		if(flag_check_start[parson_X]!=0 && flag_check_end[parson_X]!=0 && flag_check_end[parson_X] != flag_check_start[parson_X] && flag_check_end[parson_X] > flag_check_start[parson_X] && flag_check_start[parson_X]>0 &&flag_check_end[parson_X]>0){
			trans_memdata_to_pxpy(flag_check_start[parson_X]);
			human_sx[parson_X] = ppx_d;
			human_sy[parson_X] = ppy_d;
			trans_memdata_to_pxpy(flag_check_end[parson_X]);
			human_ex[parson_X] = ppx_d;
			human_ey[parson_X] = ppy_d;
			human_dr[parson_X]=hypot((human_sx[parson_X]-human_ex[parson_X]),(human_sy[parson_X]-human_ey[parson_X]));
		//������x�̕��i�R���t�B�M�����[�V�����p�����[�^�[�ύX�\�j�ړ��ŕ��̂ƔF��
			if(parson_X<=m_NUM_max){
				//int i=0;
				//for (int i = 0; i < m_NUM_max; i++) {
					//Human_Tracking::HUMAN_MIGRATION_LENGTH(i, m_qHuman);

					if (human_dr[parson_X] > m_human_width /*&&m_qHuman->human[i].velocity[1] > 0*//*|| abs(distance_xy[i][1])>0 || abs(distance_xy[i][2])>0*/ /*&& abs((m_qHuman->human[i].distance2[1] - m_qHuman->human[i].distance2[2])) <= 5*//*abs(distance_S[i])>0*/) {
						for (int i = flag_check_start[parson_X]; i <= flag_check_end[parson_X]; i++) {
							m_data->parson_divide[parson_X][i] = 1;
						}
							move_trace[parson_X][0] = flag_check_start[parson_X];
							move_trace[parson_X][1] = flag_check_end[parson_X];
							flag_check_start[parson_X] = 0;
							flag_check_end[parson_X] = 0;
							parson_X++;
					}
					//������x�̕��i�R���t�B�M�����[�V�����p�����[�^�[�ύX�\�j�ȉ��ł���ΗL���X�e�b�v�f�[�^�𖳌��ɂ���D
					else {
						for (int i = flag_check_start[parson_X]; i <= flag_check_end[parson_X]; i++) {
							m_data->parson_divide[parson_X][i] = 0;
							m_data->m_different_search[i] = 0;
						}
						flag_check_start[parson_X] = 0;
						flag_check_end[parson_X] = 0;
						move_trace[parson_X][0] = 0;
						move_trace[parson_X][1] = 0;
					}
				}
			//}
			else {}
		}
		else {}
	}


				
}
//X�CY���W�ɕϊ�����֐�
void Human_Tracking::trans_memdata_to_pxpy(int count)
{
		theta=(count-m_LRFdata_mid)*m_LRFdata_theta/360*2*M_PI/m_LRF_steps;				    //���W�A���P�ʂ̊p�x�Z�o
		G_dist=m_data->mem_data_x[count];			    //�f�[�^���ЂƂ܂������Ƃ��ĕ�
		ppx_d=1*G_dist*sin(theta);
		ppy_d=G_dist*cos(theta);



}
//�����l�ǐՊ֐�(�d�v)
void Human_Tracking::LRFPocoHuman(){

	HUMAN_STATE_CALCULATE(m_qHuman);		//�v���f�[�^�̍X�V //�Ώە�1����l�`�w10�l�ɑ΂��ČJ��Ԃ��v�Z
	TARGET_TRACKING(m_qHuman);				//�ǐՏ���
	m_human_wait = 0;
	for(int i=1 ; i<=m_NUM_max ; i++){

///////////�p�[�e�B�N���t�B���^�ɂ�郍�b�N�I������////////////////////////////////
		resample(i ,m_qHuman ,m_qHuman2);
		predict(i ,m_qHuman);
		weight(i ,m_qHuman);
		measure(i ,m_qHuman);
////////////////�p�[�e�B�N���t�B���^�ɂ�郍�b�N�I�����������܂�///////////////////

		HUMAN_MIGRATION_LENGTH(i,m_qHuman);			//�ړ������v�Z�Ɠ����ގ��J�E���g
		HUMAN_DELETE_COUNT(i,m_qHuman);             //���ގ���Ⓑ���Ԏc���Ă���l�f�[�^���폜���邽�߂ɃJ�E���g���s���D�܂��C�ǐՃG���A�ȊO�ő��x�����������̂��폜�Ώ�	
		}
	HUMAN_DELETE(m_qHuman);					        //���ގ���Ⓑ���Ԏc���Ă���l�f�[�^���폜����
	
}
//�������֐�
void Human_Tracking::Human_Initialize()
{
	m_qHuman = new Humans();
	m_qHuman2 = new Humans();
	m_data = new Data();

	ppx_d=0;
	ppy_d=0;
	theta=0;
	G_dist=0;


	for(int i=0 ; i<11 ; i++){

		for(int s2=0 ; s2<11 ; s2++){
		Tracking_check[i][s2]=0;
		m_qHuman->human[i].end=0;
		m_qHuman->human[i].start=0;
		m_qHuman->human[i].hum_count[s2]=0;

		}
		for(int i2=0;i2<max_particles;i2++){
			m_qHuman->human[i].weight[i2]=0;
			m_qHuman->human[i].xp[i2]=100000;
			m_qHuman->human[i].yp[i2]=100000;
			m_qHuman->human[i].x2[i2]=100000;
			m_qHuman->human[i].y2[i2]=100000;
			m_qHuman->human[i].x3[i2]=100000;
			m_qHuman->human[i].y3[i2]=100000;
			m_qHuman->human[i].distance2[i2]=100000;
			m_qHuman->human[i].vector[i2]=0;
			m_qHuman->human[i].velocity[i2]=0;
			tmp_x[i][i2]=0;
			tmp_y[i][i2]=0;
			tmp_x2[i][i2]=0;
			tmp_y2[i][i2]=0;
     	}
		m_qHuman->human[i].Hum_ID2=1;
		m_qHuman->human[i].Msg_Lv[1]=0;
		m_qHuman->human[i].Msg_Lv[2]=0;
		m_qHuman->human[i].Msg_Lv[3]=0;
		m_qHuman->human[i].theta[0]=0;
		m_qHuman->human[i].theta2[0]=0;
		m_qHuman->human[i].distance[0]=0;
		flag_check_start[i]=0;
		flag_check_end[i]=0;
		m_human_in=0;
		m_human_out=0;
		m_human_wait=0;
		px_d_other[i]=0;
		py_d_other[i]=0;
		theta_other[i]=0;
		G_dist_other[i]=0;
		G_senter_angl[i]=0;
			
		for(int s2=0;s2<800;s2++){
		 m_data->m_different_search[s2]=0;
		 human_sx[s2]=0;
		 human_sy[s2]=0;
		 human_ex[s2]=0;
		 human_ey[s2]=0;
		 human_dr[s2]=0;
		 m_data->parson_divide[i][s2]=0;
		 
		}
		for(int s3=0;s3<4;s3++){
		move_trace[i][s3]=0;
		distance_xy[i][s3]=0;
		distance_xy2[i][s3]=0;
		distance_xy3[i][s3]=0;
		}
	}
	for (int s3 = 0; s3 <= 10; s3++) {
		//leg_flag[s3] = 0;
		//leg_d[s3] = 0;
		//m_data->xx[s3] = 0;
		//m_data->yy[s3] = 0;
	}
	//delta_x = 0;
	//delta_y = 0;
	//vx2 = 0;
	//vx3 = 0;//�ǐՊ�̗\���ʒu


}
//�X�V�f�[�^��OUTPORT����֐�
void Human_Tracking::Data_outport()
{

	//Out�|�[�g�ɒl��n��
    m_humanX.data.length(m_NUM_max);
	for(int i=0;i<m_NUM_max;i++){
	m_humanX.data[i] = m_qHuman->human[i+1].y2[1];
	}
	m_humanXOut.write();

	m_humanY.data.length(m_NUM_max);
	for(int i=0;i<m_NUM_max;i++){
	m_humanY.data[i] = m_qHuman->human[i+1].x2[1];
	}
	m_humanYOut.write();

    m_humanV.data.length(m_NUM_max);
	for(int i=0;i<m_NUM_max;i++){
	m_humanV.data[i] = m_qHuman->human[i+1].velocity[1];
	}
	m_humanVOut.write();

	m_humanT.data.length(m_NUM_max);
	for(int i=0;i<m_NUM_max;i++){
	m_humanT.data[i] = m_qHuman->human[i+1].vector[1];
	}
	m_humanTOut.write();

	m_hcount.data.length(3);
	m_hcount.data[0] = m_human_in;
	m_hcount.data[1] = m_human_out;
	m_hcount.data[2] = m_human_wait;
	m_hcountOut.write();


	/*if(m_qHuman->human[1].distance2[1]<1000 ||m_qHuman->human[1].distance2[2]<1000 || m_qHuman->human[1].distance2[3]<1000 || m_qHuman->human[4].distance2[1]<1000 ||m_qHuman->human[5].distance2[1]<1000 || m_qHuman->human[6].distance2[1]<1000)
	{
		m_point.data.x=-2500;
		m_point.data.y=-2500;
		m_pointOut.write();
	}
	else{
		m_point.data.x=100000;
		m_point.data.y=100000;
		m_pointOut.write();
	}*/
	/*if(m_qHuman->human[1].distance2[1]<1000 ||m_qHuman->human[1].distance2[2]<1000 || m_qHuman->human[1].distance2[3]<1000 || m_qHuman->human[4].distance2[1]<1000 ||m_qHuman->human[5].distance2[1]<1000 || m_qHuman->human[6].distance2[1]<1000)
	{
		m_point.data=1;
		m_pointOut.write();
	}
	else{
		m_point.data=0;
		m_pointOut.write();
	}*/

	for(int ii=1;ii<=m_NUM_max;ii++){
		for(int iii=m_LRFdata_min;iii<=m_LRFdata_max;iii++){
		m_data->parson_divide[ii][iii]=0;
		}
		flag_check_start[ii]=0;
		flag_check_end[ii]=0;			
		move_trace[ii][0]=0;
		move_trace[ii][1]=0;
	}
	m_human_wait=0;
}
//�lID�U�蕪���֐�
void Human_Tracking::TARGET_TRACKING(struct Humans *m_MAN){

	/*
	 * �����ł͕����l�̒ǐՂ��s�����߂̏������s���Ă��܂��DLRF���瓾���镨�̃f�[�^�𐶃f�[�^�C�l�F����̈ʒu�f�[�^��l�f�[�^�Ƃ��܂��D
	 * �܂��CLRF�̍X�V�ɂ���ĕϓ�����e���f�[�^�Ɗe�l�̐l�f�[�^�̋������Z�o���܂��D
	 * ���ɁC���ꂼ��̃f�[�^���r���čŒZ�����̐��f�[�^�Ɛl�f�[�^�̑g�ݍ��킹���������܂��D
	 * �Ō�ɁC���������g�ݍ��킹�̐l�f�[�^�ɐ��f�[�^�̒l�����C�X�V���܂��D
	 * ���̍�Ƃ�l�����s���C�����l��ǐՂ��܂��D
	 *
	 */


	//�ݒ�ϐ�
	const int HUMAN_PRIME_ID[NUM_Hmax]={0,2,3,5,7,11,13,17,19,23,29};	 //�e�f��(IDHUMAN_PRIME_ID[0]�͎g��Ȃ��̂�0����)	
	double dist_check1[NUM_Hmax][NUM_Hmax]={0};							 //�e�l�Ɗe���f�[�^�Ƃ̋����L�^�p
	int hum_comp[NUM_Hmax]={0};											 //�}�b�`���O�O�̐��f�[�^�̏ꏊ�ۑ��p
	int hum_comp2=0;                                                     //�}�b�`���O�O�̐l�f�[�^�̏ꏊ�ۑ��p
	static bool c_flag[NUM_Hmax]={0};                                    //�}�b�`���O��̐��f�[�^�̏ꏊ�ۑ��p
	static bool c_flag2[NUM_Hmax]={0};                                   //�}�b�`���O��̐l�f�[�^�̏ꏊ�ۑ��p
	bool set_flag[NUM_Hmax]={0};                                         //�}�b�`���O��̐��f�[�^�̏ꏊ�ۑ��p
	bool set_flag2[NUM_Hmax]={0};                                        //�}�b�`���O��̐l�f�[�^�̏ꏊ�ۑ��p
	bool Tracking_check2[NUM_Hmax][NUM_Hmax]={0};                        //�}�b�`���O��̐l�f�[�^�y�т̐��f�[�^�̏ꏊ�ۑ��p
	static int Hum_ID[NUM_Hmax]={1,1,1,1,1,1,1,1,1,1,1};                 //�ǂ�ID���g���Ă��邩�L�^�p�i1�F�ǐՂ��Ă��Ȃ��D�f���F�ǐՒ��j

	for(int t=1;t<=m_NUM_max;t++){                                       //�ǐՂ��Ă���l�𔻕�
	Hum_ID[t]=m_qHuman->human[t].Hum_ID2;
	}	

	//----------------------------------------------------���f�[�^�Ɛl�f�[�^�}�b�`���O--------------------------------------------------------------
	for(int i=1 ; i<=m_NUM_max ; i++){
		if(Hum_ID[i]!=1){
			for(int sti=1;sti<=m_NUM_max;sti++){
				if(m_qHuman->human[sti].x[1]!=0 && m_qHuman->human[sti].y[1]!=0 ){
					dist_check1[i][sti]=(int)hypot(m_qHuman->human[i].x2[1]-m_qHuman->human[sti].x[1],m_qHuman->human[i].y2[1]-m_qHuman->human[sti].y[1]);//�e�l�Ɗe���f�[�^�Ƃ̋������L�^			    
				}	
			}
		}
	}

	for(int i=1;i<=m_NUM_max;i++){ //��
		int dist_hum[NUM_Hmax]={9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};
		for(int t=1;t<=m_NUM_max;t++){ //�l
			for(int sti=1;sti<=m_NUM_max;sti++){ //���f�[�^
				if(dist_check1[t][sti]!=0 && set_flag[sti]!=true && set_flag2[t]!=true){
					if(dist_hum[t]>dist_check1[t][sti]){
						dist_hum[t]=dist_check1[t][sti];hum_comp[t]=sti; //�e�l�ɑ΂��čŒZ�̐��f�[�^�����������f�[�^�̈ʒu���L�^
					} 
					else{}
				}
			}
		}

		double dist_hum2=9999;
		for(int t=1;t<=m_NUM_max;t++){
			if(dist_hum[t]!=0  ){
				if(dist_hum2>dist_hum[t]){dist_hum2=dist_hum[t];hum_comp2=t;} //�e�l�Ɛ��f�[�^���ŏ������̑g������
			}
		}
		if(dist_hum2!=9999){	
			m_qHuman->human[hum_comp2].xp2[1]=m_qHuman->human[hum_comp[hum_comp2]].x[1];  //�}�b�`���O��̐l�f�[�^���X�V
			m_qHuman->human[hum_comp2].yp2[1]=m_qHuman->human[hum_comp[hum_comp2]].y[1];  //�}�b�`���O��̐l�f�[�^���X�V
			m_qHuman->human[hum_comp2].end = m_qHuman->human[hum_comp2].start;            //�X�V���Ԃ�ۑ��i���x�Z�o���Ɏg�p�j
			m_qHuman->human[hum_comp2].start = clock();                                   
		if(m_qHuman->human[hum_comp2].x3[1]!=m_qHuman->human[hum_comp2].x3[2]&& m_qHuman->human[hum_comp2].x3[1]!=m_qHuman->human[hum_comp[hum_comp2]].x[1]){
			arrayInsert(m_qHuman->human[hum_comp2].x3,10,m_qHuman->human[hum_comp[hum_comp2]].x[1], 1);  //�}�b�`���O��̐l�f�[�^��ۑ�
			arrayInsert(m_qHuman->human[hum_comp2].y3,10,m_qHuman->human[hum_comp[hum_comp2]].y[1], 1);  //�}�b�`���O��̐l�f�[�^��ۑ�
		}
		set_flag[hum_comp[hum_comp2]]=true;                    //�g�p���̐��f�[�^�̏ꏊ�X�V
		set_flag2[hum_comp2]=true;                             //�g�p���̐l�f�[�^�̏ꏊ�X�V
		Tracking_check2[hum_comp2][hum_comp[hum_comp2]]=true;  //�g�p���̐��f�[�^, ���f�[�^�̏ꏊ�X�V
		}
	}
	for(int t=1;t<=m_NUM_max;t++){
			 c_flag[t]=set_flag[t];
		  	 c_flag2[t]=set_flag2[t];
			 Human_check[t]=set_flag2[t];
		 for(int s=1;s<=m_NUM_max;s++){
			 Tracking_check[t][s]=Tracking_check2[t][s];
			 }
	}
	//----------------------------------------------------���f�[�^�Ɛl�f�[�^�}�b�`���O�����܂�--------------------------------------------------------------

	//----------------------------------------------------�V�����l�������Ă����Ƃ�------------------------------------------------------------------------
	//�󂢂Ă���ID�ɐ��l����ꂢ�Ă���
   for(int sti=1 ; sti<=m_NUM_max ; sti++){
	   //�R���t�B�M�����[�V�����p�����[�^�[Human_setdistance�Őݒ肵���l�ȏ�̏ꏊ�ŕ��̂����o�����ꍇ�C�l�f�[�^�̒l���X�V
		if(m_qHuman->human[sti].y[1]!=0 && hypot(m_qHuman->human[sti].y[1],m_qHuman->human[sti].x[1])>m_Human_set&& hypot(m_qHuman->human[sti].y[1],m_qHuman->human[sti].x[1])<m_environment && c_flag[sti]!=true){
			int t=PrimeNum_Search(Hum_ID,0);//�g���Ă��Ȃ��l��T��(1�ȊO�̎g���Ă��Ȃ��f����T���D)
	        if(Hum_ID[t]==1){
			for( int i2 = 0 ; i2 < max_particles ; i2 ++ ){
				m_qHuman->human[t].xp[i2]=m_qHuman->human[sti].x[1];  //�p�[�e�B�N���̃f�[�^���X�V
				m_qHuman->human[t].yp[i2]=m_qHuman->human[sti].y[1];  //�p�[�e�B�N���̃f�[�^���X�V
				}
				m_qHuman->human[t].x2[1]=m_qHuman->human[sti].x[1]+1; //�l�f�[�^���X�V
				m_qHuman->human[t].y2[1]=m_qHuman->human[sti].y[1]+1; //�l�f�[�^���X�V
				m_qHuman->human[t].x2[0]=m_qHuman->human[sti].x[1];//�����ʒux���W�ۊ�
				m_qHuman->human[t].y2[0]=m_qHuman->human[sti].y[1];//�����ʒuy���W�ۊ�
				arrayInsert(m_qHuman->human[t].x3,10,m_qHuman->human[sti].x[1], 1); //�l�f�[�^��ۑ�
				arrayInsert(m_qHuman->human[t].y3,10,m_qHuman->human[sti].y[1], 1); //�l�f�[�^��ۑ�
				Hum_ID[t]=PrimeNum_Search(Hum_ID,1);//�g���Ă��Ȃ���ԏ�����ID�i�f���j������
				c_flag[sti]=true;
				c_flag2[t]=true;
		}
		}
	}
   //----------------------------------------------------�V�����l�������Ă����Ƃ������܂�------------------------------------------------------------------------


	for(int t=1;t<=m_NUM_max;t++){
		m_qHuman->human[t].Hum_ID2=Hum_ID[t];   //�ǐՂ��Ă���l���X�V
	}			



}
//���Ɍv���f�[�^���L�^����֐�
void Human_Tracking::HUMAN_STATE_CALCULATE(struct Humans *m_MAN){

	//���̂ƔF�����ꂽ�L���X�e�b�v�̒��ԃX�e�b�v��LRF�ɑ΂��Ă̋����Ɗp�x���Z�o�D
	for(int i=1;i<=m_NUM_max;i++){
		G_senter_angl[i]=(move_trace[i][1]+move_trace[i][0])/2;
		theta_other[i]=(G_senter_angl[i]-m_LRFdata_mid)*m_LRFdata_theta/360*2*M_PI/m_LRF_steps;						//�ڕW�̑�\�����ɂ�����p�x�����W�A���Ōv�Z
		if(G_senter_angl[i]!=0){
		G_dist_other[i]=m_data->mem_data_x[G_senter_angl[i]]; //��\�������������̋�����ݒ�
		}
		else{
			G_dist_other[i]=0;
		}
	}

	for(int i=1 ; i<=m_NUM_max ; i++){		

			//�Z�o�����v���f�[�^���L�^�i�ߋ��f�[�^��ۑ��\�D[1]�F�ŐV�f�[�^�j
			//----------------------------------------------------�ɍ��W�\��-------------------------------------------											
					arrayInsert(m_MAN->human[i].theta, 100, -theta_other[i]/M_PI*180, 1);									//�l�̊p�x����												
					arrayInsert(m_MAN->human[i].distance, 100,G_dist_other[i], 1);							                //�l�̔��a����
			//----------------------------------------------------���s���W�i���f�[�^�j---------------------------------
					arrayInsert(m_MAN->human[i].x,100,G_dist_other[i]*cos(theta_other[i]), 1);								//x���W
					arrayInsert(m_MAN->human[i].y,100,G_dist_other[i]*sin(theta_other[i]), 1);								//y���W
	}	

}
//�ړ������v������ѐl���J�E���g�֐�
void Human_Tracking::HUMAN_MIGRATION_LENGTH(int i,struct Humans *m_MAN){

	if(m_MAN->human[i].x3[1]!=100000){
	arrayInsert(m_MAN->human[i].distance2,100,hypot(m_MAN->human[i].y2[1],m_MAN->human[i].x2[1]),1);  //LRF����e�l�̋������Z�o
	arrayInsert(m_MAN->human[i].theta2,100,-atan2(m_MAN->human[i].y2[1],m_MAN->human[i].x2[1])/M_PI*180,1);  //LRF����e�l�̊p�x���Z�o
	}
        //�ǐՏ�����̒��s���W�f�[�^���瑬���ƌ������v�Z
	if(m_MAN->human[i].x3[1]!=100000&&m_MAN->human[i].x3[1]<m_environment&&m_MAN->human[i].y3[1]<m_environment){
		if(hypot((m_MAN->human[i].x3[1]-m_MAN->human[i].x3[2])/(((double)m_MAN->human[i].start-(double)m_MAN->human[i].end)/1000),(m_MAN->human[i].y3[1]-m_MAN->human[i].y3[2])/(((double)m_MAN->human[i].start-(double)m_MAN->human[i].end)/1000))<2000)
			arrayInsert(m_MAN->human[i].velocity,10,hypot((m_MAN->human[i].x3[1]-m_MAN->human[i].x3[2])/(((double)m_MAN->human[i].start-(double)m_MAN->human[i].end)/1000),(m_MAN->human[i].y3[1]-m_MAN->human[i].y3[2])/(((double)m_MAN->human[i].start-(double)m_MAN->human[i].end)/1000)),1);	
		else{}
		if(m_MAN->human[i].velocity[1]>300&&m_MAN->human[i].velocity[1]<2000){
			arrayInsert(m_MAN->human[i].vector,10,-atan2((m_MAN->human[i].y3[1]-m_MAN->human[i].y3[2]),(m_MAN->human[i].x3[1]-m_MAN->human[i].x3[2]))/M_PI*180,1);
		}
	}				
	//�l���J�E���g�����C�R���t�B�M�����[�V�����p�����[�^�[Count_mode��ύX���邱�Ƃœ��ގ��̌�����ύX�\�D
	if(m_Count_mode=="IN,OUT"){
		if(m_MAN->human[i].distance2[1]>m_Count_area && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]>0 && m_MAN->human[i].Msg_Lv[2]!=3){
			m_MAN->human[i].Msg_Lv[2]=1;
		}
		else if(m_MAN->human[i].distance2[1]>m_Count_area && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]<0 && m_MAN->human[i].Msg_Lv[2]!=3){
			m_MAN->human[i].Msg_Lv[2]=2;
		}
		else{}

		if(m_MAN->human[i].Msg_Lv[2] == 1 && m_MAN->human[i].distance2[1]>m_Count_area-500 && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]<0){
			m_human_in++;
			m_MAN->human[i].Msg_Lv[2]=3;
					
		}else if(m_MAN->human[i].Msg_Lv[2]==2 && m_MAN->human[i].distance2[1]>m_Count_area-500 && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]>0){
			m_human_out++;
			m_MAN->human[i].Msg_Lv[2]=3;
		}
		else{}
	}
	else if (m_Count_mode=="OUT,IN"){
		if(m_MAN->human[i].distance2[1]>m_Count_area && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]<0 && m_MAN->human[i].Msg_Lv[2]!=3){
			m_MAN->human[i].Msg_Lv[2]=1;
		}
		else if(m_MAN->human[i].distance2[1]>m_Count_area && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]>0 && m_MAN->human[i].Msg_Lv[2]!=3){
			m_MAN->human[i].Msg_Lv[2]=2;
		}
		else{}

		if(m_MAN->human[i].Msg_Lv[2] == 1 && m_MAN->human[i].distance2[1]>m_Count_area-500 && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]>0){
			m_human_in++;
			m_MAN->human[i].Msg_Lv[2]=3;
					
		}else if(m_MAN->human[i].Msg_Lv[2]==2 && m_MAN->human[i].distance2[1]>m_Count_area-500 && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]<0){
			m_human_out++;
			m_MAN->human[i].Msg_Lv[2]=3;
		}
		else{}
	}
	else{
		if(m_MAN->human[i].distance2[1]>m_Count_area && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]>0 && m_MAN->human[i].Msg_Lv[2]!=3){
			m_MAN->human[i].Msg_Lv[2]=1;
		}
		else if(m_MAN->human[i].distance2[1]>m_Count_area && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]<0 && m_MAN->human[i].Msg_Lv[2]!=3){
			m_MAN->human[i].Msg_Lv[2]=2;
		}
		else{}

		if(m_MAN->human[i].Msg_Lv[2] == 1 && m_MAN->human[i].distance2[1]>m_Count_area-500 && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]<0){
			m_human_in++;
			m_MAN->human[i].Msg_Lv[2]=3;
					
		}else if(m_MAN->human[i].Msg_Lv[2]==2 && m_MAN->human[i].distance2[1]>m_Count_area-500 && m_MAN->human[i].distance2[1]<m_environment && m_qHuman->human[i].theta2[1]>0){
			m_human_out++;
			m_MAN->human[i].Msg_Lv[2]=3;
		}
		else{}
	}
	if(m_qHuman->human[i].Hum_ID2!=1)m_human_wait++;  //�؍ݐl�����J�E���g
	
}
//���ގ���Ⓑ���Ԏc������l�f�[�^�̎c�����Ԃ��J�E���g����֐�
void Human_Tracking::HUMAN_DELETE_COUNT(int i,struct Humans *m_MAN){

	if(m_MAN->human[i].velocity[1]==m_MAN->human[i].velocity[2]&& m_MAN->human[i].distance2[1]<m_environment)m_MAN->human[i].hum_count[1]++;
	if(m_MAN->human[i].distance2[1]>2500 && m_MAN->human[i].distance2[1]<m_environment && m_MAN->human[i].velocity[1]<180)m_MAN->human[i].hum_count[1]++;
	else if(m_MAN->human[i].distance2[1]<2500)m_MAN->human[i].hum_count[1]=0; 
	else if(m_MAN->human[i].velocity[1]>180 && m_MAN->human[i].velocity[1]!=m_MAN->human[i].velocity[2])m_MAN->human[i].hum_count[1]=0;
	else{}
	if(m_MAN->human[i].distance2[1]<2500 && m_MAN->human[i].velocity[1]<180 )m_MAN->human[i].hum_count[2]++;
	else if(m_MAN->human[i].velocity[1]>180)m_MAN->human[i].hum_count[2]=0;
	else{}
	if(m_MAN->human[i].distance2[1]>m_environment)m_MAN->human[i].hum_count[3]++;
	if(m_MAN->human[i].velocity[1]==m_MAN->human[i].velocity[2]&& m_MAN->human[i].distance2[1]<m_environment)m_MAN->human[i].hum_count[4]++;
	if(m_MAN->human[i].x3[1]<0 && m_MAN->human[i].velocity[1]<180)m_MAN->human[i].hum_count[4]++;
	else if(m_MAN->human[i].x3[1]>0)m_MAN->human[i].hum_count[4]=0; 
	else if(m_MAN->human[i].velocity[1]>180 && m_MAN->human[i].velocity[1]!=m_MAN->human[i].velocity[2])m_MAN->human[i].hum_count[4]=0;
	else{}
	if(m_MAN->human[i].x2[1]<0)m_MAN->human[i].hum_count[5]++; 
	else if(m_MAN->human[i].x2[1]>0)m_MAN->human[i].hum_count[5]=0;
	
}
//�l�f�[�^���폜����֐�
void Human_Tracking::HUMAN_DELETE(struct Humans *m_MAN){

	for(int i=1 ; i<=m_NUM_max ; i++){
		for (int count = m_data->LRF_start; count < m_data->LRF_end; count++) {
			
			if (m_MAN->human[i].hum_count[1] >= 10 || m_MAN->human[i].hum_count[2] > 2000 || m_MAN->human[i].hum_count[3] > 10 || m_MAN->human[i].hum_count[4] > 20 /*&& m_data->parson_divide[i][count]!=1*/ || m_MAN->human[i].hum_count[5] > 100 /*&& m_data->m_different_search[count]!=1*//*&&flag_check_start[i]==0 &&flag_check_end[i]==0*/) {
				for (int s2 = 1; s2 <= 11; s2++) {
					m_MAN->human[i].hum_count[s2] = 0;
				}
				for (int i2 = 0; i2 < max_particles; i2++) {
					m_MAN->human[i].weight[i2] = 0;
					m_MAN->human[i].xp[i2] = 100000;
					m_MAN->human[i].yp[i2] = 100000;
					m_MAN->human[i].x2[i2] = 100000;
					m_MAN->human[i].y2[i2] = 100000;
					m_MAN->human[i].x2[i2] = 100000;
					m_MAN->human[i].y3[i2] = 100000;
					m_MAN->human[i].distance2[i2] = 100000;
					m_MAN->human[i].vector[i2] = 0;
					m_MAN->human[i].velocity[i2] = 0;
				}
				m_MAN->human[i].Hum_ID2 = 1;
				m_MAN->human[i].Msg_Lv[1] = 0;
				m_MAN->human[i].Msg_Lv[2] = 0;
				m_MAN->human[i].Msg_Lv[3] = 0;
				m_MAN->human[i].theta[0] = 0;
				m_MAN->human[i].theta2[0] = 0;
				m_MAN->human[i].distance[0] = 0;
			}
		}
	}
}
//�v���������Ƃ̃f�[�^���c���֐�/��ԌÂ��f�[�^�������ĐV�������l�ɍX�V����֐��i[1]:�ŐV�f�[�^�j
void Human_Tracking::arrayInsert(double data[], int n, double newdata, int k)
{
    int i ;
	
    for (i = n; i > k; i--) {
        data[i] = data[i-1];
		    }
    data[k] = newdata;
	
}
//���g�pID(�f��)�T���֐�
int Human_Tracking::PrimeNum_Search(int HUMAN_ID[],int MODE){
		//const int N=7;
		int ANS=1;
		const int SEARCH_NUM[NUM_Hmax]={0,2,3,5,7,11,13,17,19,23,29};//��������f��[0]�͌������Ȃ�
		int Discovery_NUM=0;//����������������֐�
			//�S�Ă�ID���|����
			for(int i=1;i<=NUM_Hmax;i++){
				if(ANS*HUMAN_ID[i]!=0)ANS*=HUMAN_ID[i];	
			}
			//�f������
			for(int i=1;i<=NUM_Hmax;i++){
				if(ANS%SEARCH_NUM[i]!=0){
					if(MODE==0)Discovery_NUM=i;//���݂��Ȃ��f���ň�ԏ������l���L�^�i�����Ɍ������邱�Ƃ��\�j
					else Discovery_NUM=SEARCH_NUM[i];//���݂��Ȃ��f���ň�ԏ������lID���L�^�i�����Ɍ������邱�Ƃ��\�j
					break;
				}
			}
 return Discovery_NUM;//�g���Ă��Ȃ�ID�ň�ԏ������̂�Ԃ�
}

//�p�[�e�B�N���t�B���^�ɂ�郍�b�N�I������
void Human_Tracking::resample( int i, struct Humans *m_MAN,struct Humans *m_MAN2)
{
    // �ݐϏd�݂̌v�Z
	double all_weight[max_particles]={0.0};
	all_weight[0]=m_MAN->human[i].weight[0];
	for(int i2=1;i2<max_particles;i2++)
	{
		all_weight[i2]=all_weight[i2-1]+m_MAN->human[i].weight[i2];
		
	}
    // �d�݂���Ƀp�[�e�B�N�������T���v�����O���ďd�݂�1.0��
	for(int i2=0;i2<max_particles;i2++)
	{
		m_MAN2->human[i].weight[i2] = m_MAN->human[i].weight[i2];
		m_MAN2->human[i].xp[i2] = m_MAN->human[i].xp[i2];
		m_MAN2->human[i].yp[i2] = m_MAN->human[i].yp[i2];
	}
    for( int i2 = 0 ; i2 < max_particles; i2 ++ )
    {
		double rate=(double)(rand()%100)/100;
		if(rate<cut_rate)rate+=(1.0-cut_rate);
        const double weight = rate * all_weight[max_particles-1];
        size_t n = 0;
        while( all_weight[++n] < weight );
		m_MAN->human[i].weight[i2]=m_MAN2->human[i].weight[n];
		m_MAN->human[i].xp[i2]=m_MAN2->human[i].xp[n];
		m_MAN->human[i].yp[i2]=m_MAN2->human[i].yp[n];
        m_MAN->human[i].weight[i2] = 1.0;
    }



}
void Human_Tracking::predict(int i, struct Humans *m_MAN)
{
	double variance =0;
	int sum =0;
	//���̒l���ړ������̐ώZ�Ƌ��ɑ��傳���悤
	if(m_MAN->human[i].Hum_ID2!=1){
		for(int s=1;s<=m_NUM_max;s++){
			sum += Tracking_check[i][s];
		}
		if(sum==0){
			// �ʒu�̗\��
			for( size_t i2 = 0 ; i2 < max_particles ; i2 ++ )
			{
				if(m_MAN->human[i].velocity[1]>300 && m_MAN->human[i].distance2[1]<m_Human_TrackingData){
					double rate=0.0;
					rate=(double)(rand()%100-20);
					double vx2= m_MAN->human[i].velocity[1]*(((double)m_MAN->human[i].start-(double)m_MAN->human[i].end)/1000)*cos(m_qHuman->human[i].vector[1]*M_PI/180.0)/20.0*rate;
					rate=(double)(rand()%100-20);
					double vx3= -m_MAN->human[i].velocity[1]*(((double)m_MAN->human[i].start-(double)m_MAN->human[i].end)/1000)*sin(m_qHuman->human[i].vector[1]*M_PI/180.0)/20.0*rate;			
					m_MAN->human[i].xp[i2] += vx2;
					m_MAN->human[i].yp[i2] += vx3;
				}
				else {
					variance = 4.0;
					double rate=0.0;
					rate=(double)(rand()%200-100);
					double vx2= variance*rate;
					rate=(double)(rand()%200-100);
					double vx3= variance*rate;
					m_MAN->human[i].xp[i2] += vx2;
					m_MAN->human[i].yp[i2] += vx3;

				}
				
			}
		}
		else if(sum==1){
				if(m_MAN->human[i].velocity[1]<200){
					variance = 1.0;
				}
				else if(200<m_MAN->human[i].velocity[1] ){
					variance = 2.0;
				}
				else{}
				if(m_MAN->human[i].velocity[1]>700||m_MAN->human[i].velocity[1]==m_MAN->human[i].velocity[2]){
					variance = 4.0;
				}
				else{}

    
					// �ʒu�̗\��
				for( size_t i2 = 0 ; i2 < max_particles ; i2 ++ )
				{
					double rate=0.0;
					rate=(double)(rand()%200-100);
					double vx2= variance*rate;
					rate=(double)(rand()%200-100);
					double vx3= variance*rate;
					m_MAN->human[i].xp[i2] += vx2;
					m_MAN->human[i].yp[i2] += vx3;	
				}
		}
		else{}
	}
	else{}

}
void Human_Tracking::weight( int i, struct Humans *m_MAN)
{
    // �ޓx�ɏ]���p�[�e�B�N���̏d�݂����肷��
	double max_l_dist=0.0,heading=9999;
	double lrf_weight[max_particles];


	for(int i2=0;i2<max_particles;i2++)
	{
		lrf_weight[i2]=9999;		
		double dist=0.0;
		double delta_x=m_qHuman->human[i].xp2[1]-m_MAN->human[i].xp[i2];
		double delta_y=m_qHuman->human[i].yp2[1]-m_MAN->human[i].yp[i2];
		dist=hypot(delta_x,delta_y);
		if(lrf_weight[i2]>dist)lrf_weight[i2]=dist;		
	}
	
	for(int i2=0;i2<max_particles;i2++)
	{
		if(lrf_weight[i2]>max_l_dist) max_l_dist=lrf_weight[i2];
	}
	for(int i2=0;i2<max_particles;i2++)
	{
		m_MAN->human[i].weight[i2]=1-lrf_weight[i2]/max_l_dist;
		
	}
}
void Human_Tracking::measure(int i, struct Humans *m_MAN)
{
    double  x = 0.0;
    double  y = 0.0;
    double  weight = 0.0;

    // �d�ݘa
    for( size_t i2 = 0 ; i2 < max_particles ; i2 ++ )
    {
        x       += m_MAN->human[i].xp[i2]         * m_MAN->human[i].weight[i2];
        y       += m_MAN->human[i].yp[i2]         * m_MAN->human[i].weight[i2];
        weight  += m_MAN->human[i].weight[i2];
    }
    // ���K��
	if(weight!=0){
	arrayInsert(m_qHuman->human[i].x2,100,static_cast< int >( x / weight ), 1);
	arrayInsert(m_qHuman->human[i].y2,100,static_cast< int >( y / weight ), 1);
	}
	for(int i2=0;i2<max_particles;i2++){
		tmp_x[i][i2]=m_MAN->human[i].xp[i2];
		tmp_y[i][i2]=m_MAN->human[i].yp[i2];
		}
}



/*
RTC::ReturnCode_t Human_Tracking::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Human_Tracking::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Human_TrackingInit(RTC::Manager* manager)
  {
    coil::Properties profile(human_tracking_spec);
    manager->registerFactory(profile,
                             RTC::Create<Human_Tracking>,
                             RTC::Delete<Human_Tracking>);
  }
  
};


