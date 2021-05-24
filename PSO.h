#pragma once
#ifndef _PSO
#define _PSO

#include <stdlib.h>
#include <iostream>
#include <vector>
#include "FitnessFunction.h"
#include "main.h"

// ��Ӧ����Խ��Խ�û���ԽСԽ��
//#define MINIMIZE_FITNESS
#define MAXIMIZE_FITNESS

struct PSOPara
{
	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�
	int particle_num_;					// ���Ӹ���
	int max_iter_num_;					// ����������

	double *dt_ = nullptr;							// ʱ�䲽��
	double *wstart_ = nullptr;						// ��ʼȨ��
	double *wend_ = nullptr;						// ��ֹȨ��
	double *C1_ = nullptr;							// ���ٶ�����
	double *C2_ = nullptr;							// ���ٶ�����

	double *upper_bound_ = nullptr;					// position������Χ����
	double *lower_bound_ = nullptr;					// position������Χ����
	double *range_interval_ = nullptr;				// position�������䳤��
	
	int results_dim_ = 0;								// results��ά��

	PSOPara(){}

	PSOPara(int dim, bool hasBound = false)
	{
		dim_ = dim;

		dt_ = new double[dim_];
		wstart_ = new double[dim_];
		wend_ = new double[dim_];
		C1_ = new double[dim_];
		C2_ = new double[dim_];
		if (hasBound)
		{
			upper_bound_ = new double[dim_];
			lower_bound_ = new double[dim_];
			range_interval_ = new double[dim_];
		}
	}

	// �����������ͷŶ��ڴ�
	~PSOPara()
	{
		if (upper_bound_) { delete[]upper_bound_; }
		if (lower_bound_) { delete[]lower_bound_; }
		if (range_interval_) { delete[]range_interval_; }
		if (dt_) { delete[]dt_; }
		if (wstart_) { delete[]wstart_; }
		if (wend_) { delete[]wend_; }
		if (C1_) { delete[]C1_; }
		if (C2_) { delete[]C2_; }
	}
};

struct Particle
{
	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�
	double fitness_;
	double *position_ = nullptr;
	double *velocity_ = nullptr;

	double *best_position_ = nullptr;
	double best_fitness_;
	double *results_ = nullptr;			// һЩ��Ҫ������Ľ��
	int results_dim_ = 0;				// results_��ά��

	Particle(){}

	~Particle()
	{
		if (position_) { delete[]position_; }
		if (velocity_) { delete[]velocity_; }
		if (best_position_) { delete[]best_position_; }
		if (results_) { delete[]results_; }
	}

	Particle(int dim, double *position, double *velocity, double *best_position, double best_fitness);
};

typedef double(*ComputeFitness)(Particle& particle);

class PSOOptimizer
{
public:
	int particle_num_;					// ���Ӹ���
	int max_iter_num_;					// ����������
	int curr_iter_;						// ��ǰ��������
	int n;
	int currentpoints;

	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�
	float* mapangle;

	Particle *particles_ = nullptr;		// ��������
	
	double *upper_bound_ = nullptr;					// position������Χ����
	double *lower_bound_ = nullptr;					// position������Χ����
	double *range_interval_ = nullptr;				// position�������䳤��

	double *dt_ = nullptr;							// ʱ�䲽��
	double *wstart_ = nullptr;						// ��ʼȨ��
	double *wend_ = nullptr;						// ��ֹȨ��
	double *w_ = nullptr;							// ��ǰ����Ȩ��
	double *C1_ = nullptr;							// ���ٶ�����
	double *C2_ = nullptr;							// ���ٶ�����

	double all_best_fitness_;						// ȫ���������ӵ���Ӧ��ֵ
	double *all_best_position_ = nullptr;			// ȫ���������ӵ�poistion
	double *results_ = nullptr;						// һЩ��Ҫ������Ľ��
	int results_dim_ = 0;							// results��ά��

	ComputeFitness fitness_fun_ = nullptr;			// ��Ӧ�Ⱥ���

public:
	// Ĭ�Ϲ��캯��
	PSOOptimizer() {}

	// ���캯��
	PSOOptimizer(PSOPara* pso_para);
	
	// ��������
	~PSOOptimizer();

	// ��ʼ���������Ӳ���
	void InitialAllParticles(float camangle);

	// ��ʼ����i�����Ӳ���
	void InitialParticle(int i,float camangle);

	// ��ȡ˫�����������Ĭ�Ͼ���Ϊ0.0001��
	double GetDoubleRand(int N = 9999);

	// ��������ӵ���Ӧ��ֵ
	double GetFitness(Particle& particle,float camangle);

	// �����������Ӳ���
	void UpdateAllParticles(float camangle);

	// ���µ�i������
	void UpdateParticle(int i,float camangle);

	// ��ȡ��ǰ������Ȩ��
	void GetInertialWeight();

	void GetNum(int a,float* b);

	bool CmpAngle(float angle, float yx, float qua);
};
#endif