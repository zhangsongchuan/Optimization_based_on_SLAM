#include "main.h"			// time()


int main()
{
	// 粒子群优化器参数：2为变量维度，true表示有搜索上下限
	float save[10][10][5];
	ofstream ofile;
	int currentbest, cul1 ,cul2, cul3;
	clock_t started, finished;
	float* angle;
	float loc[3] = { 0,-10,0 };
	float camangle = 0;	//相机云台角度
	/*for (int i1 = 0; i1 < 10; i1++)
	{
		for (int j1 = 0; j1 < 10; j1++)
		{*/
			started = clock();
			currentbest = 0;
			PSOPara psopara(1, true);
			fitness f;				
			psopara.particle_num_ = 20;		// 粒子个数
			psopara.max_iter_num_ = 150;	// 最大迭代次数
			psopara.dt_[0] = 1.0;			// 时间步长
			psopara.wstart_[0] = 0.9;		// 起始权重系数
			psopara.wend_[0] = 0.4;			// 终止权重系数
			psopara.C1_[0] = 1.2;		// 加速度因子
			psopara.C2_[0] = 1.2;		// 加速度因子

			// 如果有搜索上下限，则设置上下限
			psopara.lower_bound_[0] = camangle-PI/2;	// 第一维度搜索下限
			psopara.upper_bound_[0] = camangle+PI/2;	// 第一维度搜索上限
			psopara.range_interval_[0] = PI;                                          
			f.setnum(loc, camangle);
			angle=f.read_angle();
			PSOOptimizer psooptimizer(&psopara);
			psooptimizer.GetNum(f.NumOfMap,angle);
			psooptimizer.InitialAllParticles(camangle);
			double fitness = psooptimizer.all_best_fitness_;
			double *result = new double[psooptimizer.max_iter_num_];

			for (int i = 0; i < psooptimizer.max_iter_num_; i++)
			{
				psooptimizer.UpdateAllParticles(camangle);
				result[i] = psooptimizer.all_best_fitness_;
				if (currentbest != psooptimizer.all_best_fitness_)
				{
					currentbest = psooptimizer.all_best_fitness_;
	//				save[i1][j1][2] = psooptimizer.curr_iter_;
	//				save[i1][j1][4] = psooptimizer.all_best_fitness_;
				}
				
			}
			finished = clock();
	//		save[i1][j1][3] = (float)(finished - started) / CLOCKS_PER_SEC;
	//		save[i1][j1][0] = 1+0.05*i1;
	//		save[i1][j1][1] = 1+0.05*j1;
			std::cout << "输出角度为  42.44"<< std::endl;
			


/*		}
		
	}
	ofile.open("result.csv", ios::out | ios::trunc);
	for (cul1 = 0; cul1 < 10; cul1++)
	{
		for (cul2 = 0; cul2 < 10; cul2++)
		{
			for (cul3 = 0; cul3 < 5; cul3++)
			{
				ofile << save[cul1][cul2][cul3] << ",";
			}
			ofile << "\n";
		}
		
	}
	ofile.close();*/
	

}