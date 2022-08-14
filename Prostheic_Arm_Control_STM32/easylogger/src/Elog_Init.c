#include "Elog_Init.h"

void Elog_Init()
{
/* 初始化elog */
elog_init();
elog_set_text_color_enabled(true);

/* 设置每个级别的日志输出格式 */
//输出所有内容
elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
//输出日志级别信息和日志TAG
elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG);
elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG);
elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG);
//除了时间、进程信息、线程信息之外，其余全部输出
elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_T_INFO));
//输出所有内容
elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);

/* 启动elog */
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
	