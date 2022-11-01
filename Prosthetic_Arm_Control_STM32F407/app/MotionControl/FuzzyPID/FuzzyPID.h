#ifndef __FUZZYPID_H
#define __FUZZYPID_H

#ifdef __cplusplus
extern "C" {
#endif

class FuzzyPID
{
	public:
    FuzzyPID();
    ~FuzzyPID();
		void Get_grad_membership(float erro,float erro_c);//输入e与de/dt隶属度计算函数///
		void GetSumGrad();//获取输出增量kp,ki,kd的总隶属度/
		void GetOUT();//计算输出增量kp,kd,ki对应论域值//
		float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float erro, float erro_c,float ki_max,float ki_min,float kd_max,float kd_min,float erro_pre,float errp_ppre);
		float Quantization(float maximum,float minimum,float x);//区间映射函数
		float Inverse_quantization(float maximum, float minimum, float qvalues);
		int changeOut(double out,double sss);
	
		const int  num_area = 8; //划分区域个数
		float e_membership_values[7]  = { -3,-2,-1,0,1,2,3 };//输入E(目标位姿与实际位姿的偏差)的隶属值
    float ec_membership_values[7] = { -3,-2,-1,0,1,2,3 };//输入dE/dt（当前偏差和上次偏差的差值）的隶属值
		float kp_menbership_values[7] = { -3,-2,-1,0,1,2,3 };//输出增量kp的隶属值
    float ki_menbership_values[7] = { -3,-2,-1,0,1,2,3 }; //输出增量ki的隶属值
    float kd_menbership_values[7] = { -3,-2,-1,0,1,2,3 };  //输出增量kd的隶属值
		
		float kp,ki,kd;//PID参数
		float qdetail_kp;               //增量kp对应论域中的值
    float qdetail_ki;               //增量ki对应论域中的值
    float qdetail_kd;               //增量kd对应论域中的值
    float qfuzzy_output;
    float detail_kp;                //输出增量kp
    float detail_ki;                //输出增量ki
    float detail_kd;                //输出增量kd
    float fuzzy_output;
		float erro,erro_c;//误差、误差变化量
    float qerro;                    //输入e对应论域中的值
    float qerro_c;                  //输入de/dt对应论域中的值
    float errosum;
    float e_gradmembership[2];      //输入e的隶属度
    float ec_gradmembership[2];     //输入de/dt的隶属度
    int e_grad_index[2];            //输入e隶属度在规则表的索引
    int ec_grad_index[2];           //输入de/dt隶属度在规则表的索引
    float gradSums[7] = {0,0,0,0,0,0,0};
    float KpgradSums[7] = { 0,0,0,0,0,0,0 };   //输出增量kp总的隶属度
    float KigradSums[7] = { 0,0,0,0,0,0,0 };   //输出增量ki总的隶属度
    float KdgradSums[7] = { 0,0,0,0,0,0,0 };   //输出增量kd总的隶属度
		
		
//		float integral,maxIntegral;//积分、积分限幅
//		float fuzzy_output,maxOutput,minOutput;//输出、输出限幅
	
		int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //论域隶属值

		int  Kp_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},     //kp规则表
                                {PB,PB,PB,PM,PM,ZO,ZO},
                                {PB,PM,PM,PS,ZO,NS,NM},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NM,NM,NM,NB},
                                {ZO,ZO,ZO,NM,NB,NB,NB},
                                {ZO,NS,NB,NB,NB,NB,NB} };

		int  Ki_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},     //kp规则表
                                {PB,PB,PB,PM,PM,ZO,ZO},
                                {PB,PM,PM,PS,ZO,NS,NM},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NM,NM,NM,NB},
                                {ZO,ZO,ZO,NM,NB,NB,NB},
                                {ZO,NS,NB,NB,NB,NB,NB} };

		int  Kd_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},     //kp规则表
                                {PB,PB,PB,PM,PM,ZO,ZO},
                                {PB,PM,PM,PS,ZO,NS,NM},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NM,NM,NM,NB},
                                {ZO,ZO,ZO,NM,NB,NB,NB},
                                {ZO,NS,NB,NB,NB,NB,NB} };
};
	
#ifdef __cplusplus
}
#endif

#endif