#include "FitnessFunction.h"



void fitness::setnum(float* location, float cam_angle)		//设定内参数，即相机云台位置及朝向
{
	for (int i = 0; i < 3; i++)
		position[i] = location[i];
	angle = cam_angle;
}

float* fitness::read_angle()		//读取地图点并计算出角度
{
	std::ifstream f;
	int i, j;
	unsigned long int n;
	float xyz;
	float x, y;
	
	f.open("D:\\map.bin", std::ios::binary);
	f.seekg(0, ios::beg);
	f.read((char*)&n, sizeof(n));
	NumOfMap = n;
	for (i = 0; i < n; i++)
	{
		f.read((char*)&xyz, sizeof(float));
		if (xyz >= 20 || xyz < -20)
			xyz = rand() % 10+1;
		mappoints[i][0] = xyz;
		f.read((char*)&xyz, sizeof(float));
		if (xyz > 20 || xyz < -20)
			xyz = rand() % 10 + 1;
		mappoints[i][1] = xyz;
		f.read((char*)&xyz, sizeof(float));
		if (xyz > 20 || xyz < -20)
			xyz = rand() % 10 + 1;
		mappoints[i][2] = xyz;
	}
	f.close();
	for (i = 0; i < n; i++,i++)
	{
		x = mappoints[i][0]-position[0];
		y = mappoints[i][1]-position[1];
		if (x > 0 && y >= 0)
		{
			mapangle[i+0] = y / x;
			mapangle[i+1] = 1;
		}
			
		else if (x > 0 && y < 0)
		{
			mapangle[i+0] = y / x;
			mapangle[i+1] = 4;

		}
		else if (x < 0 && y > 0)
		{
			mapangle[i+0] = y / x;
			mapangle[i+1] = 2;
		}
		else if (x < 0 && y <= 0)
		{
			mapangle[i+0] = y / x;
			mapangle[i+1] = 3;
		}
		else if (x == 0 && y > 0)
		{
			mapangle[i+0] = 100000;
			mapangle[i+1] = 1;
		}
		else
		{
			mapangle[i+0] = -100000;
			mapangle[i+1] = 4;
		}


	}
	return mapangle;
}

