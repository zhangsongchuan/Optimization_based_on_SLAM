#include "PSO.h"

// 构造函数
PSOOptimizer::PSOOptimizer(PSOPara* pso_para)
{
	particle_num_ = pso_para->particle_num_;
	max_iter_num_ = pso_para->max_iter_num_;
	dim_ = pso_para->dim_;
	curr_iter_ = 0;

	dt_ = new double[dim_];
	wstart_ = new double[dim_];
	wend_ = new double[dim_];
	C1_ = new double[dim_];
	C2_ = new double[dim_];

	for (int i = 0; i < dim_; i++)
	{
		dt_[i] = pso_para->dt_[i];
		wstart_[i] = pso_para->wstart_[i];
		wend_[i] = pso_para->wend_[i];
		C1_[i] = pso_para->C1_[i];
		C2_[i] = pso_para->C2_[i];
	}

	if (pso_para->upper_bound_ && pso_para->lower_bound_)
	{
		upper_bound_ = new double[dim_];
		lower_bound_ = new double[dim_];
		range_interval_ = new double[dim_];

		for (int i = 0; i < dim_; i++)
		{
			upper_bound_[i] = pso_para->upper_bound_[i];
			lower_bound_[i] = pso_para->lower_bound_[i];
			//range_interval_[i] = pso_para.range_interval_[i];
			range_interval_[i] = upper_bound_[i] - lower_bound_[i];
		}
	}

	particles_ = new Particle[particle_num_];
	w_ = new double[dim_];
	all_best_position_ = new double[dim_];

	results_dim_ = pso_para->results_dim_;

	if (results_dim_)
	{
		results_ = new double[results_dim_];
	}

}

PSOOptimizer::~PSOOptimizer()
{
	if (particles_) { delete[]particles_; }
	if (upper_bound_) { delete[]upper_bound_; }
	if (lower_bound_) { delete[]lower_bound_; }
	if (range_interval_) { delete[]range_interval_; }
	if (dt_) { delete[]dt_; }
	if (wstart_) { delete[]wstart_; }
	if (wend_) { delete[]wend_; }
	if (w_) { delete[]w_; }
	if (C1_) { delete[]C1_; }
	if (C2_) { delete[]C2_; }
	if (all_best_position_) { delete[]all_best_position_; }
	if (results_) { delete[]results_; }
}

// 初始化所有粒子
void PSOOptimizer::InitialAllParticles(float camangle)
{
	// 初始化第一个粒子参数并设置最优值
	InitialParticle(0,camangle);
	all_best_fitness_ = particles_[0].best_fitness_;
	for (int j = 0; j < dim_; j++)
	{
		all_best_position_[j] = particles_[0].best_position_[j];
	}

	// 初始化其他粒子，并更新最优值
	for (int i = 1; i < particle_num_; i++)
	{
		InitialParticle(i,camangle);
#ifdef MAXIMIZE_FITNESS
		if (particles_[i].best_fitness_ > all_best_fitness_)
#else
		if (particles_[i].best_fitness_ < all_best_fitness_)
#endif
		{
			all_best_fitness_ = particles_[i].best_fitness_;
			for (int j = 0; j < dim_; j++)
			{
				all_best_position_[j] = particles_[i].best_position_[j];
			}

			// 如果需要保存出一些结果
			if (particles_[i].results_dim_ && results_dim_ == particles_[i].results_dim_)
			{
				for (int k = 0; k < results_dim_; k++)
				{
					results_[k] = particles_[i].results_[k];
				}
			}
			else if (results_dim_)
			{
				std::cout << "WARNING: the dimension of your saved results for every particle\nis not match with the dimension you specified for PSO optimizer ant no result is saved!" << std::endl;
			}
		}
	}
}

// 获取双精度随机数
double PSOOptimizer::GetDoubleRand(int N)
{
	double temp = rand() % (N + 1) / (double)(N + 1);
	return temp;
}

double PSOOptimizer::GetFitness(Particle & particle, float camangle)		//在此计算适应度
{
	float angle1,punish1=0,punish2=0,final,tannum;
	angle1=particle.position_[0];
	int num = 0,i;
	
	float canshu = 0;
	for (i = 0; i < n; i++)
	{
		

		if ((!CmpAngle(angle1-0.74,mapangle[2*i], mapangle[2 * i+1]))&&(CmpAngle(angle1 + 0.74, mapangle[2 * i], mapangle[2 * i + 1])))
			num = num + 1;
	}
	final = num ;
	return final;
}

bool  PSOOptimizer::CmpAngle(float angle , float yx , float qua)		//比较角度大小，1为大
{
	int quadrant = 0;
	if (angle >= 0 && angle <= PI / 2)
		quadrant = 1;
	if (angle > PI / 2 && angle < PI)
		quadrant = 2;
	if (angle >= -PI && angle < -PI / 2)
		quadrant = 3;
	if (angle >= -PI / 2 && angle < 0)
		quadrant = 4;
	if (quadrant > qua)
		return 1;
	else if (quadrant < qua)
		return 0;
	else
	{
		if (quadrant == 1 || quadrant == 4)
			if (tan(angle) > yx)
				return 1;
			else
				return 0;
		if (quadrant == 2)
			if (tan(angle - PI) > yx)
				return 1;
			else
				return 0;
		if (quadrant == 3)
			if (tan(angle + PI) > yx)
				return 1;
			else
				return 0;
	}

}

void PSOOptimizer::UpdateAllParticles(float camangle)
{
	GetInertialWeight();
	for (int i = 0; i < particle_num_; i++)
	{
		UpdateParticle(i,camangle);
#ifdef MAXIMIZE_FITNESS
		if (particles_[i].best_fitness_ > all_best_fitness_)
#else
		if (particles_[i].best_fitness_ < all_best_fitness_)
#endif
		{
			all_best_fitness_ = particles_[i].best_fitness_;
			for (int j = 0; j < dim_; j++)
			{
				all_best_position_[j] = particles_[i].best_position_[j];
			}
			
			// 如果需要保存出一些参数
			if (particles_[i].results_dim_ && results_dim_ == particles_[i].results_dim_)
			{
				for (int k = 0; k < results_dim_; k++)
				{
					results_[k] = particles_[i].results_[k];
				}
			}
			else if (results_dim_)
			{
				std::cout << "WARNING: the dimension of your saved results for every particle\nis not match with the dimension you specified for PSO optimizer ant no result is saved!" << std::endl;
			}
		}
	}
	curr_iter_++;
}

void PSOOptimizer::UpdateParticle(int i, float camangle)
{
	// 计算当前迭代的权重
	for (int j = 0; j < dim_; j++)
	{
		// 保存上一次迭代结果的position和velocity
		//double last_velocity = particles_[i].velocity_[j];
		double last_position = particles_[i].position_[j];

		particles_[i].velocity_[j] = w_[j] * particles_[i].velocity_[j] +
			C1_[j] * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
			C2_[j] * GetDoubleRand() * (all_best_position_[j] - particles_[i].position_[j]);
		particles_[i].position_[j] += dt_[j] * particles_[i].velocity_[j];

		// 如果搜索区间有上下限限制
		if (upper_bound_ && lower_bound_)
		{
			if (particles_[i].position_[j] > upper_bound_[j])
			{
				double thre = GetDoubleRand(99);
				if (last_position == upper_bound_[j])
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = upper_bound_[j] - (upper_bound_[j] - last_position) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = upper_bound_[j];
				}		
			}
			if (particles_[i].position_[j] < lower_bound_[j])
			{
				double thre = GetDoubleRand(99);
				if (last_position == lower_bound_[j])
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = lower_bound_[j] + (last_position - lower_bound_[j]) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = lower_bound_[j];
				}
			}
		}
	}
	particles_[i].fitness_ = GetFitness(particles_[i], camangle);

#ifdef MAXIMIZE_FITNESS
	if (particles_[i].fitness_ > particles_[i].best_fitness_)
#else
	if (particles_[i].fitness_ < particles_[i].best_fitness_)
#endif
	{
		particles_[i].best_fitness_ = particles_[i].fitness_;
		for (int j = 0; j < dim_; j++)
		{
			particles_[i].best_position_[j] = particles_[i].position_[j];
		}
	}
}


void PSOOptimizer::GetInertialWeight()
{
	double temp = log((double)curr_iter_) / log((double)max_iter_num_);
	temp *= temp;
	for (int i = 0; i < dim_; i++)
	{
		w_[i] = wstart_[i] - (wstart_[i] - wend_[i]) * temp;
	}
}

void PSOOptimizer::GetNum(int a, float* b)
{
	n = a;
	mapangle = b;
}


void PSOOptimizer::InitialParticle(int i,float camangle)
{
	// 为每个粒子动态分配内存
	particles_[i].position_ = new double[dim_];
	particles_[i].velocity_ = new double[dim_];
	particles_[i].best_position_ = new double[dim_];

	//if (results_dim_)
	//{
	//	particles_[i].results_ = new double[results_dim_];
	//}

	// 初始化position/veloctiy值
	for (int j = 0; j < dim_; j++)
	{
		// if defines lower bound and upper bound
		if (range_interval_)
		{
			particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
			particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
//			std::cout << particles_[i].position_[j] << std::endl;
		}
		else
		{
			particles_[i].position_[j] = GetDoubleRand() * 2;
			particles_[i].velocity_[j] = GetDoubleRand() * 0.5;
		}
	}

	// 设置初始化最优适应度值
	particles_[i].fitness_ = GetFitness(particles_[i],camangle);

	for (int j = 0; j < dim_; j++)
	{
		particles_[i].best_position_[j] = particles_[i].position_[j];
	}
	particles_[i].best_fitness_ = particles_[i].fitness_;
}

// 此函数未用到
Particle::Particle(int dim, double * position, double * velocity, double * best_position, double best_fitness)
{
	dim_ = dim;
	//position_ = new double[dim];
	//velocity_ = new double[dim];
	//best_position_ = new double[dim];
	position_ = position;
	velocity_ = velocity;
	best_position_ = best_position;
	best_fitness_ = best_fitness;
}
