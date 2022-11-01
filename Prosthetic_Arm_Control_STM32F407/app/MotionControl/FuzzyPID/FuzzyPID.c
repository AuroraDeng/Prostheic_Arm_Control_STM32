#include "FuzzyPID.h"
#include "math.h"
FuzzyPID::FuzzyPID()  //构造函数
{
    kp = 0;
    ki = 0;
    kd = 0;
    fuzzy_output = 0;
    qdetail_kp = 0;
    qdetail_ki = 0;
    qdetail_kd = 0;
    qfuzzy_output = 0;
    errosum = 0;
}

FuzzyPID::~FuzzyPID()//析构函数
{
	
}

//输入e与de/dt隶属度计算函数///
void FuzzyPID::Get_grad_membership(float erro,float erro_c)
{
	//计算误差的隶属度
    if (erro > e_membership_values[0] && erro < e_membership_values[6])//在误差项NB~PB的范围内
    {
        for (int i = 0; i < num_area - 2; i++)
        {
            if (erro >= e_membership_values[i] && erro <= e_membership_values[i + 1])//判断误差属于哪个范围
            {

                //隶属度计算
                e_gradmembership[0] = -(erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                e_gradmembership[1] = 1+(erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);

                //所属区间
                e_grad_index[0] = i;
                e_grad_index[1] = i + 1;
                break;
            }
        }
    }
    else
    {
        if (erro <= e_membership_values[0])
        {
            e_gradmembership[0] = 1;
            e_gradmembership[1] = 0;
            e_grad_index[0] = 0;
            e_grad_index[1] = -1;
        }
        else if (erro >= e_membership_values[6])
        {
            e_gradmembership[0] = 1;
            e_gradmembership[1] = 0;
            e_grad_index[0] = 6;
            e_grad_index[1] = -1;
        }
    }

		//计算误差变化量的隶属度
    if (erro_c > ec_membership_values[0] && erro_c < ec_membership_values[6])
    {
        for (int i = 0; i < num_area - 2; i++)
        {
            if (erro_c >= ec_membership_values[i] && erro_c <= ec_membership_values[i + 1])
            {
                ec_gradmembership[0] = -(erro_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
                ec_gradmembership[1] = 1 + (erro_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
							
                ec_grad_index[0] = i;
                ec_grad_index[1] = i + 1;
                break;
            }
        }
    }
    else
    {
        if (erro_c <= ec_membership_values[0])
        {
            ec_gradmembership[0] = 1;
            ec_gradmembership[1] = 0;
            ec_grad_index[0] = 0;
            ec_grad_index[1] = -1;
        }
        else if (erro_c >= ec_membership_values[6])
        {
            ec_gradmembership[0] = 1;
            ec_gradmembership[1] = 0;
            ec_grad_index[0] = 6;
            ec_grad_index[1] = -1;
        }
    }
}

//获取输出增量kp,ki,kd的总隶属度/
void FuzzyPID::GetSumGrad()
{
    //连续计算时清空
    for (int i = 0; i <7; i++)
    {
        KpgradSums[i] = 0;
        KigradSums[i] = 0;
        KdgradSums[i] = 0;
    }

   //两两组合，四种情况相累计
  for (int i=0;i<2;i++)
  {
      if (e_grad_index[i] == -1)
      {
       continue;
      }
      for (int j = 0; j < 2; j++)
      {
          if (ec_grad_index[j] != -1)
          {
              //规则从属
              int indexKp = Kp_rule_list[ec_grad_index[j]][e_grad_index[i]] + 3;
              int indexKi = Ki_rule_list[ec_grad_index[j]][e_grad_index[i]] + 3;
              int indexKd = Kd_rule_list[ec_grad_index[j]][e_grad_index[i]] + 3;
              //gradSums[index] = gradSums[index] + (e_gradmembership[i] * ec_gradmembership[j])* Kp_rule_list[e_grad_index[i]][ec_grad_index[j]];

              //根据从属计算隶属度
              KpgradSums[indexKp]= KpgradSums[indexKp] + (e_gradmembership[i] * ec_gradmembership[j]);
              KigradSums[indexKi] = KigradSums[indexKi] + (e_gradmembership[i] * ec_gradmembership[j]);
              KdgradSums[indexKd] = KdgradSums[indexKd] + (e_gradmembership[i] * ec_gradmembership[j]);
          }
          else
          {
            continue;
          }

      }
  }
}

//计算输出增量kp,kd,ki对应论域值//
void FuzzyPID::GetOUT()
{
    for (int i = 0; i < num_area - 1; i++)
    {
        qdetail_kp += kp_menbership_values[i] * KpgradSums[i];
        qdetail_ki += ki_menbership_values[i] * KigradSums[i];
        qdetail_kd+= kd_menbership_values[i] * KdgradSums[i];
    }
}

int FuzzyPID::changeOut(double out,double sss)
{
    int times=0;
    int a=0;
    if(sss<0)
    {
        a=1;
    }else {
        a=0;
    }
    if(out>0)
    {
        times = out/0.53+a;
    }else{
        times = out/0.1-1;
    }
    return times;
}

//模糊PID控制实现函数/
float FuzzyPID::FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float erro, float erro_c,float ki_max,float ki_min,float kd_max,float kd_min,float erro_pre,float errp_ppre)
{
    errosum += erro;//误差累积
    qerro = Quantization(e_max, e_min, erro);
    qerro_c = Quantization(ec_max, ec_min, erro_c);
    Get_grad_membership(qerro, qerro_c);
    GetSumGrad();
    GetOUT();
    detail_kp = Inverse_quantization(kp_max, kp_min, qdetail_kp);
    detail_ki = Inverse_quantization(ki_max, ki_min, qdetail_ki);
    detail_kd = Inverse_quantization(kd_max, kd_min, qdetail_kd);
    qdetail_kd = 0;
    qdetail_ki = 0;
    qdetail_kp = 0;
    kp = kp + detail_kp;
    ki = ki + detail_ki;
    kd = kd + detail_kd;
  detail_kp = 0;
  detail_ki=0;
  detail_kd=0;
  //qDebug()<<"error:"<<erro<<"\t"<<"erro_pre:"<<erro_pre;
  double sss = erro;
  erro=abs(erro);
  erro_pre=abs(erro_pre);
  errp_ppre= abs(errp_ppre);
  //qDebug()<<"erro:"<<erro<<"\t"<<"erro_pre:"<<erro_pre<<"\t"<<"errp_ppre:"<<errp_ppre;
  float output = kp * erro + ki * (erro - 2 * erro_pre + errp_ppre) + kd*(erro - erro_pre);
  output=changeOut(output,sss);
  //qDebug()<<"kp:"<<kp<<"\t"<<"ki:"<<ki<<"\t"<<"kd:"<<kd<<"out:"<<output;
  return output;
}

//区间映射函数
float FuzzyPID::Quantization(float maximum,float minimum,float x)
{
    float qvalues= 6.0 *(x-minimum)/(maximum - minimum)-3;
    return qvalues;
}

//反区间映射函数
float FuzzyPID::Inverse_quantization(float maximum, float minimum, float qvalues)
{
    float x = (maximum - minimum) *(qvalues + 3)/6 + minimum;
    return x;
}