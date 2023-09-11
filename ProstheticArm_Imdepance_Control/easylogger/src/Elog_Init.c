#include "Elog_Init.h"

void Elog_Init()
{
/* ��ʼ��elog */
elog_init();
elog_set_text_color_enabled(true);

/* ����ÿ���������־�����ʽ */
//�����������
elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
//�����־������Ϣ����־TAG
elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG);
elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG);
elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG);
//����ʱ�䡢������Ϣ���߳���Ϣ֮�⣬����ȫ�����
elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_T_INFO));
//�����������
elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);

/* ����elog */
elog_start();

}

void elos_test()
{
	elog_a("main","Hello EasyLogger!");
	elog_e("main","Hello EasyLogger!");
	elog_w("main","Hello EasyLogger!");
	elog_i("main","Hello EasyLogger!");
	elog_d("main","Hello EasyLogger!");
	elog_v("main","Hello EasyLogger!");
}
	