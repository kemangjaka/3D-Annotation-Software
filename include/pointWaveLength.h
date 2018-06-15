#pragma once
#define PCL_NO_PRECOMPILE


#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
using namespace pcl;
struct PointWaveLength
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	unsigned char nir;
	unsigned char ndvi;
	unsigned short wvl1;
	unsigned short wvl2;
	unsigned short wvl3;
	unsigned short wvl4;
	unsigned short wvl5;
	unsigned short wvl6;
	unsigned short wvl7;
	int label;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointWaveLength,
(float, x, x)
(float, y, y)
(float, z, z)
(float, rgb, rgb)
(int, label, label)
(unsigned char, nir, nir)
(unsigned char, ndvi, ndvi)
(unsigned short, wvl1, wvl1)
(unsigned short, wvl2, wvl2)
(unsigned short, wvl3, wvl3)
(unsigned short, wvl4, wvl4)
(unsigned short, wvl5, wvl5)
(unsigned short, wvl6, wvl6)
(unsigned short, wvl7, wvl7)
)
PCL_INSTANTIATE(KdTreeFLANN, PointWaveLength)
