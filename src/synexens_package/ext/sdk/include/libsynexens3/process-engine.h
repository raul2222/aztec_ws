/**
 * @file process-engine.h
 * @author synexens
 * @brief 
 * @version 0.0.4
 * @date 2022-04-24
 * 
 * Copyright(c) 2022 synexens.Co.Ltd. All rights reserved
 * 
 */
#ifndef LIBSYNEXENS3_PROCESSING_ENGINE_H
#define LIBSYNEXENS3_PROCESSING_ENGINE_H

#include "macros.h"
#include "types.h"
#include "frame.h"

namespace SY3_NAMESPACE
{

	class SY3_EXPORT process_engine
	{

	public:
		/**
		 * \brief 计算点云
		 * \param[in] depth 深度帧
		 * \return points，需要手动释放内存
		 */
		virtual points *comptute_points(depth_frame *depth,sy3_error &error) =0;

		/**
		 * \brief rgbd对齐
		 * \param[in] depth depth帧
		 * \param[in] rgb  rgb帧
		 * \return frameset，需要手动释放内存
		 */
		virtual sy3::frameset *align_to_rgb(depth_frame *depth, rgb_frame *rgb,sy3_error &error) =0;


		 /*
		 *	深度滤波
		 *  深度指针
		 */
		virtual void depth_filter(uint16_t* depth, sy3::FilterType filter_type) = 0;

                /*
		*  depth深度数据
		*  width 深度宽度
		*  height深度高度
		*  undistort_depth去畸变之后的深度
		*  isTof是否是深度相机（TOF或者RGB相机都可去畸变）
		*/
		virtual void undistort(const uint16_t* depth, int width, int height, uint16_t* undistort_depth, bool isTof) = 0;

	};

};

#endif