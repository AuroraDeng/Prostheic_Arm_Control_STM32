int i; 
union xxx{char m[4]; float n;uint32_t r;int j;}z;

for (i = 0; i < 4; i++)  //一维扭矩传感器
{
	z.m[i] = ReceivedBuffer[2 + i];
}
Torque = z.j * 0.0004;

for (i = 0; i < 4; i++)  //一维扭矩传感器反序
{
	z.m[i] = ReceivedBuffer[6 - i];
}
Torque = z.j * 0.0004;
//////////////////////////////////////////////////////////
for (i = 0; i < 4; i++)  //六维长数据传感器
{
	z.m[i] = ReceivedBuffer[2 + i];
}
Fx = z.n;
for (i = 0; i < 4; i++)
{
	z.m[i] = ReceivedBuffer[6 + i];
}
Fy = z.n;
for (i = 0; i < 4; i++)
{
	z.m[i] = ReceivedBuffer[10 + i];
}
Fz = z.n;
for (i = 0; i < 4; i++)
{
	z.m[i] = ReceivedBuffer[14 + i];
}
Mx = z.n;
for (i = 0; i < 4; i++)
{
	z.m[i] = ReceivedBuffer[18 + i];
}
My = z.n;
for (i = 0; i < 4; i++)
{
	z.m[i] = ReceivedBuffer[22 + i];
}
Mz = z.n;