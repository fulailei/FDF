#include "HcPathPlanningCore.h"
#include "HcPathPlanningInterface.h"
#include "HcPathPlanningUtils.h"

#include "HcIKInterface.h"
#include "HcInterpolationInterface.h"
#include "HcCollisionDetectionInterface.h"
#include "HcCatchPointSortInterface.h"

#include <vector>
#include <ctime>


PATH_PLANNING_API U32 HcPathPlanning::hcPathPlanningGetVersion()
{
	U32    version;
	U32    date;

	version = (HC_PATH_PLANNING_MAJOR_VERSION << 10)
		| (HC_PATH_PLANNING_MINOR_VERSION << 5)
		| (HC_PATH_PLANNING_REVISION_VERSION);
	date = (HC_PATH_PLANNING_VER_YEAR << 9)
		| (HC_PATH_PLANNING_VER_MONTH << 5)
		| (HC_PATH_PLANNING_VER_DAY);

	return ((version << 16) | date);
}

PATH_PLANNING_API S32 HcPathPlanning::hcPathPlanningCreate(IN void **ppcf_handle, IN HcPathPlanningCfgParams *ppcf_cfg_params)
{
	if (NULL == ppcf_handle)
	{
		return HC_PATH_PLANNING_E_PARAM_NULL;
	}

	if (NULL == ppcf_cfg_params ||
		NULL == ppcf_cfg_params->envir_file_path ||
		NULL == ppcf_cfg_params->longaxiz_file_path ||
		NULL == ppcf_cfg_params->path_param_file_path ||
		NULL == ppcf_cfg_params->robot_file_path ||
		NULL == ppcf_cfg_params->workpiece_file_path)
	{
		return HC_PATH_PLANNING_E_CFG_INVALID;
	}

	if (ppcf_cfg_params->is_use_seven_axis)
	{
		if (NULL == ppcf_cfg_params->shortaxiz_file_path)
		{
			return HC_PATH_PLANNING_E_CFG_INVALID;
		}
	}

	if (ppcf_cfg_params->is_detect_box)
	{
		if (NULL == ppcf_cfg_params->box_file_path)
		{
			return HC_PATH_PLANNING_E_CFG_INVALID;
		}
	}


	//分配内存空间
	S32 pcf_struct_size = sizeof(HcPathPlanningStruct);
	U08 *pstruct_mem = (U08 *)malloc(pcf_struct_size * sizeof(U08));
	if (NULL == pstruct_mem)
	{
		return HC_PATH_PLANNING_E_MEM_SMALL;
	}
	memset(pstruct_mem, 0, pcf_struct_size);
	*ppcf_handle = pstruct_mem;
	HcPathPlanningStruct *path_planning_struct = (HcPathPlanningStruct *)(*ppcf_handle);
	path_planning_struct->is_seven_axiz = ppcf_cfg_params->is_use_seven_axis;
	path_planning_struct->is_detect_box = ppcf_cfg_params->is_detect_box;

	const U32 model_data_max = 50000; //模型数据上限
	path_planning_struct->box_mesh_data = new F32[model_data_max];
	memset(path_planning_struct->box_mesh_data, 0, model_data_max * sizeof(F32));
	path_planning_struct->envir_mesh_data = new F32[model_data_max];
	memset(path_planning_struct->envir_mesh_data, 0, model_data_max * sizeof(F32));
	path_planning_struct->long_axiz_mesh_data = new F32[model_data_max];
	memset(path_planning_struct->long_axiz_mesh_data, 0, model_data_max * sizeof(F32));
	path_planning_struct->short_axiz_mesh_data = new F32[model_data_max];
	memset(path_planning_struct->short_axiz_mesh_data, 0, model_data_max * sizeof(F32));
	path_planning_struct->workpiece_mesh_data = new F32[10*model_data_max];
	memset(path_planning_struct->workpiece_mesh_data, 0, 10*model_data_max * sizeof(F32));
	path_planning_struct->concat_axis_data = new F32[10 * model_data_max];
	memset(path_planning_struct->concat_axis_data, 0, 10 * model_data_max * sizeof(F32));
	//读配置文件

	bool b_ret_1 = hcReadRobotConfig(ppcf_cfg_params->robot_file_path, *path_planning_struct);
	bool b_ret_2 = hcReadPathPlanConfig(ppcf_cfg_params->path_param_file_path, *path_planning_struct);

	bool b_ret_3 = true;
	if (ppcf_cfg_params->is_detect_box)
	{
		b_ret_3 = readFile(ppcf_cfg_params->box_file_path,
			path_planning_struct->box_mesh_data,
			path_planning_struct->box_mesh_size);
	}
	bool b_ret_4 = readFile(ppcf_cfg_params->envir_file_path,
		path_planning_struct->envir_mesh_data,
		path_planning_struct->envir_mesh_size);
	bool b_ret_5 = readFile(ppcf_cfg_params->longaxiz_file_path,
		path_planning_struct->long_axiz_mesh_data,
		path_planning_struct->long_axiz_mesh_size);
	bool b_ret_6 = true;
	if (ppcf_cfg_params->is_use_seven_axis)
	{
	    b_ret_6 = readFile(ppcf_cfg_params->shortaxiz_file_path,
			path_planning_struct->short_axiz_mesh_data,
			path_planning_struct->short_axiz_mesh_size);
	}
	bool b_ret_7 = readFile(ppcf_cfg_params->workpiece_file_path,
		path_planning_struct->workpiece_mesh_data,
		path_planning_struct->workpiece_mesh_size);

	if (!(b_ret_1&&b_ret_2&&b_ret_3&&b_ret_4&&b_ret_5&&b_ret_6&&b_ret_7))
	{
		return HC_PATH_PLANNING_E_CFG_INVALID;
	}

	//saveMatToPCD(path_planning_struct->workpiece_mesh_data,
	//	path_planning_struct->workpiece_mesh_size, "workpiece.pcd");


	//创建逆解模块句柄
	HcIK::hcIKCreate(&path_planning_struct->ik_ppcf_handle);
	//创建插补模块句柄
	HcInterpolation::hcInterpolationCreate(&path_planning_struct->interpolation_ppcf_handle);
	//创建抓取点排序模块句柄
	HcCatchPointSort::hcCatchPointSortCreate(&path_planning_struct->sort_ppdf_handle);
	//创建碰撞检测模块句柄
	HcCollisionDetection::hcCollisionDetectionCreate(&path_planning_struct->collision_ppcf_handle);

	return HANCHINE_S_OK;
}

PATH_PLANNING_API S32 HcPathPlanning::hcPathPlanningSetParams(IN void *ppcf_handle, IN HcPathPlanningDynParams *ppcf_dyn_params)
{
	if (NULL == ppcf_handle)
	{
		return HC_PATH_PLANNING_E_PARAM_NULL;
	}

	if (NULL == ppcf_dyn_params )
	{
		return HC_PATH_PLANNING_E_DYN_INVALID;
	}

	HcPathPlanningStruct *path_planning_struct = (HcPathPlanningStruct *)ppcf_handle;

	//设置参数
#pragma region 解析参数
	HcPathPlanParamList param_list = ppcf_dyn_params->param_list;
	for (U32 i = 0; i < param_list.size; i++)
	{
		U32 index = param_list.param[i].key;
		F32 value = param_list.param[i].value;
		switch (index)
		{
		case EnumHcToolParamTCPx:
			path_planning_struct->TCP.x = value;
			break;
		case EnumHcToolParamTCPy:
			path_planning_struct->TCP.y = value;
			break;
		case EnumHcToolParamTCPz:
			path_planning_struct->TCP.z = value;
			break;
		case EnumHcToolParamTCPEulerx:
			path_planning_struct->TCP.euler_x = value;
			break;
		case EnumHcToolParamTCPEulery:
			path_planning_struct->TCP.euler_y = value;
			break;
		case EnumHcToolParamTCPEulerz:
			path_planning_struct->TCP.euler_z = value;
			break;
		case EnumHcToolParamLongAxizx:
			path_planning_struct->tool_long_axiz[0] = value;
			break;
		case EnumHcToolParamLongAxizy:
			path_planning_struct->tool_long_axiz[1] = value;
			break;
		case EnumHcToolParamLongAxizz:
			path_planning_struct->tool_long_axiz[2] = value;
			break;
		case EnumHcToolParamRotationAxizx:
			path_planning_struct->tool_rotation_axiz[0] = value;
			break;
		case EnumHcToolParamRotationAxizy:
			path_planning_struct->tool_rotation_axiz[1] = value;
			break;
		case EnumHcToolParamRotationAxizz:
			path_planning_struct->tool_rotation_axiz[2] = value;
			break;
		case EnumHcToolParamShortAxizx:
			path_planning_struct->tool_short_axiz[0] = value;
			break;
		case EnumHcToolParamShortAxizy:
			path_planning_struct->tool_short_axiz[1] = value;
			break;
		case EnumHcToolParamShortAxizz:
			path_planning_struct->tool_short_axiz[2] = value;
			break;
		case EnumHcToolParamMaxAngle:
			path_planning_struct->tool_max_anlge = value;
			break;
		case EnumHcToolParamMinAngle:
			path_planning_struct->tool_min_angle = value;
			break;
		case EnumHcToolParamPointNum:
			path_planning_struct->tool_angle_num = value;
			break;
		case EnumHcFirstPointHeight:
			path_planning_struct->first_point_height = value;
			break;
		case EnumHcApproachLength:
			path_planning_struct->approach_length = value;
			break;
		case EnumHcCollisionPointNum:
			path_planning_struct->detected_collision_point_num = value;
			break;
		case EnumHcBoxPosex:
			path_planning_struct->BOX.x = value;
			break;
		case EnumHcBoxPosey:
			path_planning_struct->BOX.y = value;
			break;
		case EnumHcBoxPosez:
			path_planning_struct->BOX.z = value;
			break;
		case EnumHcBoxPoseEulerx:
			path_planning_struct->BOX.euler_x = value;
			break;
		case EnumHcBoxPoseEulery:
			path_planning_struct->BOX.euler_y = value;
			break;
		case EnumHcBoxPoseEulerz:
			path_planning_struct->BOX.euler_z = value;
			break;
		case EnumHcPathType:
			path_planning_struct->path_type = EnnumHcPathPlanningType((int)value);
			break;
		case EnumHcCollisionLevel:
			path_planning_struct->collision_level = EnumHcCollisionAccuracy((int)value);
			break;
		case EnumHcRunningState:
			path_planning_struct->running_model = EnumHcRunningModel((int)value);
			break;
		case EnumHcCatchSortType:
			path_planning_struct->catch_policy = EnumHcCatchPolicy((int)value);
			break;
		case EnumHcPathNum:
			path_planning_struct->path_num_max = value;
			break;
		default:
			break;
		}
	}
#pragma endregion
	//保存抓取点
	if (ppcf_dyn_params->catch_point_list.size > 0)
	{
		path_planning_struct->vec_catch_point.clear();
		for (S32 i = 0; i < ppcf_dyn_params->catch_point_list.size; i++)
		{
			path_planning_struct->vec_catch_point.push_back(ppcf_dyn_params->catch_point_list.point_data[i]);
		}
	}

	//设置逆解模块动态参数
	HcIKDynParams ik_dyn_params;
	ik_dyn_params.DH = path_planning_struct->DH;
	ik_dyn_params.type = EnumHcPose2RotQuaternion;
	HcIK::hcIKSetParams(
		path_planning_struct->ik_ppcf_handle, &ik_dyn_params);

	//设置插补模块动态参数
	HcInterpolationDynParams interpolation_dyn_params;
	interpolation_dyn_params.interpolation_typy = EnumHcLineInterpoaltion;
	HcInterpolation::hcInterpolationSetParams(
		path_planning_struct->interpolation_ppcf_handle, &interpolation_dyn_params);

	//设置抓取点排序模块动态参数
	HcCatchPointSortDynParams sort_dyn_params;
	sort_dyn_params.catch_policy = path_planning_struct->catch_policy;
	HcCatchPointSort::hcCatchPointSortSetParams(
		path_planning_struct->sort_ppdf_handle, &sort_dyn_params);

	//设置碰撞检测模块动态参数
	HcCollisionDetectionDynParams collision_dyn_params;
	//添加料框数据
	if (path_planning_struct->is_detect_box)
	{
		Eigen::Matrix4f box_pose = poseToRT(
			path_planning_struct->BOX, RZXYn);
		transPose(path_planning_struct->box_mesh_data,
			path_planning_struct->box_mesh_size, box_pose.data());
		collision_dyn_params.max_layer = 16;
		collision_dyn_params.object_type = EnumHcBoxData;
		collision_dyn_params.data = path_planning_struct->box_mesh_data;
		collision_dyn_params.size = path_planning_struct->box_mesh_size;
		HcCollisionDetection::hcCollisionDetectionSetParams(
			path_planning_struct->collision_ppcf_handle, &collision_dyn_params);

		//saveMatToPCD(path_planning_struct->box_mesh_data,
		//	path_planning_struct->box_mesh_size, "box.pcd");
	}


	//saveMatToPCD(path_planning_struct->box_mesh_data,
	//	path_planning_struct->box_mesh_size,
	//	"./boxtrans.pcd");

	//添加环境数据
	collision_dyn_params.max_layer = 16;
	collision_dyn_params.object_type = EnumHcEnvir;
	collision_dyn_params.data = path_planning_struct->envir_mesh_data;
	collision_dyn_params.size = path_planning_struct->envir_mesh_size;
	HcCollisionDetection::hcCollisionDetectionSetParams(
	path_planning_struct->collision_ppcf_handle, &collision_dyn_params);
	//添加工件数据
	collision_dyn_params.max_layer = 16;
	collision_dyn_params.object_type = EnumHcWorkpiece;
	collision_dyn_params.data = path_planning_struct->workpiece_mesh_data;
	collision_dyn_params.size = path_planning_struct->workpiece_mesh_size;
	HcCollisionDetection::hcCollisionDetectionSetParams(
		path_planning_struct->collision_ppcf_handle, &collision_dyn_params);
	//添加轴数据
	if (path_planning_struct->is_seven_axiz)   //有第七轴
	{
		path_planning_struct->concat_axis_size =
		path_planning_struct->long_axiz_mesh_size + path_planning_struct->short_axiz_mesh_size;
		path_planning_struct->vec_axis_angle.clear();
		path_planning_struct->vec_axis_angle = divisionAngle(
		path_planning_struct->tool_min_angle,
		path_planning_struct->tool_max_anlge,
		path_planning_struct->tool_angle_num);
		path_planning_struct->vec_TCP.clear();
		path_planning_struct->vec_axis_pose.clear();
		for (S32 i = 0; i < path_planning_struct->tool_angle_num; i++)
		{
			path_planning_struct->vec_TCP.push_back(
				getTCP(
					path_planning_struct->tool_long_axiz,
					path_planning_struct->tool_rotation_axiz,
					path_planning_struct->tool_short_axiz,
					path_planning_struct->vec_axis_angle[i]));
			path_planning_struct->vec_axis_pose.push_back(
				getShortAxizPosition(
					path_planning_struct->tool_long_axiz,
					path_planning_struct->tool_rotation_axiz,
					path_planning_struct->tool_short_axiz,
					path_planning_struct->vec_axis_angle[i]));

			concatShortAxisAndLongAxis(
				path_planning_struct->short_axiz_mesh_data,
				path_planning_struct->short_axiz_mesh_size,
				path_planning_struct->long_axiz_mesh_data,
				path_planning_struct->long_axiz_mesh_size,
				path_planning_struct->concat_axis_data,
				path_planning_struct->vec_axis_pose);

			collision_dyn_params.max_layer = 16;
			collision_dyn_params.object_type = EnumHcAxisData;
			collision_dyn_params.axis_num = path_planning_struct->tool_angle_num;
			collision_dyn_params.data = path_planning_struct->concat_axis_data;
			collision_dyn_params.size = path_planning_struct->concat_axis_size;
			HcCollisionDetection::hcCollisionDetectionSetParams(
				path_planning_struct->collision_ppcf_handle, &collision_dyn_params);

			//saveMatToPCD(path_planning_struct->concat_axis_data,
			//	path_planning_struct->concat_axis_size, "axis.pcd");
		}
	}
	else  //没第七轴   YZX
	{
		path_planning_struct->vec_TCP.clear();
		path_planning_struct->vec_TCP.push_back(
			poseToRT(path_planning_struct->TCP, RZXYn));

		collision_dyn_params.max_layer = 16;
		collision_dyn_params.object_type = EnumHcAxisData;
		collision_dyn_params.axis_num = 1;
		collision_dyn_params.data = path_planning_struct->long_axiz_mesh_data;
		collision_dyn_params.size = path_planning_struct->long_axiz_mesh_size;
		HcCollisionDetection::hcCollisionDetectionSetParams(
			path_planning_struct->collision_ppcf_handle, &collision_dyn_params);
	}

	return HANCHINE_S_OK;
}

PATH_PLANNING_API S32 HcPathPlanning::hcPathPlanningProcessing(IN void *ppcf_handle, OUT HcPathPlanningProcessParams *ppcf_process_params)
{
	if (NULL == ppcf_handle)
	{
		return HC_PATH_PLANNING_E_PARAM_NULL;
	}

	if (NULL == ppcf_process_params ||
		NULL == ppcf_process_params->cloud_data ||
		0 > ppcf_process_params->pose_list.size)
	{
		return HC_PATH_PLANNING_E_PRC_INVALID;
	}

	HcPathPlanningStruct *path_planning_struct = (HcPathPlanningStruct *)ppcf_handle;
	S32 process_state = HANCHINE_S_OK;
	clock_t startTime, endTime;

	//点云转无序滤除NAN点
	startTime = clock();
	float* cloud_data = new float[3 * ppcf_process_params->cloud_size];
	memset(cloud_data, 0, 3 * ppcf_process_params->cloud_size * sizeof(float));
	float cloud_x = 0;
	float cloud_y = 0;
	float cloud_z = 0;
	int cloud_size = 0;
	for (int i = 0; i < ppcf_process_params->cloud_size; i++)
	{
		cloud_x = ppcf_process_params->cloud_data[3 * i + 0];
		cloud_y = ppcf_process_params->cloud_data[3 * i + 1];
		cloud_z = ppcf_process_params->cloud_data[3 * i + 2];
		if (std::isnan(cloud_x) || std::isnan(cloud_y) || std::isnan(cloud_z))
		{
			continue;
		}
		if (cloud_x == 0 && cloud_y == 0 && cloud_z == 0)
		{
			continue;
		}
		cloud_data[3 * cloud_size + 0] = cloud_x;
		cloud_data[3 * cloud_size + 1] = cloud_y;
		cloud_data[3 * cloud_size + 2] = cloud_z;
		cloud_size++;
	}
	endTime = clock();
	//std::cout << "remove nan time:" << double(endTime - startTime) << "ms" << std::endl;

	//saveMatToPCD(cloud_data,
	//	cloud_size, "scan.pcd");

	//设置碰撞检测模块动态参数
	HcCollisionDetectionDynParams collision_dyn_params;
	collision_dyn_params.max_layer = 16;
	collision_dyn_params.object_type = EnumHcScanData;
	collision_dyn_params.data = cloud_data;
	collision_dyn_params.size = cloud_size;
	HcCollisionDetection::hcCollisionDetectionSetParams(
		path_planning_struct->collision_ppcf_handle, &collision_dyn_params);

	delete[] cloud_data;

	//业务流程
	S32 workpiece_index[500] = { 0 };
	path_planning_struct->vec_all_path.clear();
	path_planning_struct->vec_ok_path.clear();
	path_planning_struct->path_point.clear();
	path_planning_struct->interpolation_path_point.clear();
	path_planning_struct->blacklist.clear();
	path_planning_struct->robot_mat.clear();
	std::vector<std::vector<HcRobotPose>> interpolation_path_point = path_planning_struct->interpolation_path_point;

	S32 curr_point_index = 0;
	S32 curr_path_index = 0;
	U32 point_size = 0;

	//生成抓取点列表
	startTime = clock();
	std::vector<HcRobotRT> generate_catch_point_list;
	HcCatchPointSortProcessParams sort_process_params;
	sort_process_params.catch_point_list = (void*)&path_planning_struct->vec_catch_point;
	sort_process_params.post_list = ppcf_process_params->pose_list;
	S32 nRet = HcCatchPointSort::hcCatchPointSortProcessing(
		path_planning_struct->sort_ppdf_handle, &sort_process_params);
	generate_catch_point_list = *(std::vector<HcRobotRT>*)sort_process_params.sort_chatch_point;
	endTime = clock();
	//std::cout << "generate_catch_point_list time:" << double(endTime - startTime) << "ms" << std::endl;

	//生成轨迹点
	startTime = clock();
	vector<HcPathData> vec_path_data;
	path_planning_struct->path_point =
		generatePathPoint(generate_catch_point_list,
			path_planning_struct->vec_TCP,
			path_planning_struct->path_type,
			path_planning_struct->approach_length,
			path_planning_struct->first_point_height,
			vec_path_data,
			path_planning_struct->catch_point_perpendicularity);
	endTime = clock();
	//std::cout << "generate path_point time:" << double(endTime - startTime) << "ms" << std::endl;

	startTime = clock();
	//生成插补点
	//
	HcRobotPose goalpoint;
	HcRobotPose startpoint;
	startpoint.joint[0] = 1.72;
	startpoint.joint[1] = 0.45;
	startpoint.joint[2] = -0.45;
	startpoint.joint[3] = 0.034;
	startpoint.joint[4] = 1.7;
	startpoint.joint[5] = 1.67;

	//
	if (EnumHcThreePoint == path_planning_struct->path_type)
	{
		point_size = 40;
		for (curr_path_index = 0; curr_path_index < path_planning_struct->path_point.size(); curr_path_index++)
		{
			//三点法

			S32 path_ok_num = 0;
			HcPath path;
			memset(&path, 0, sizeof(HcPath));
			path.path_status = EnumHcPathOK;
			path.workpiece_index = vec_path_data[curr_path_index].workpiece_id;    //工件编号
			path.catch_point_index = vec_path_data[curr_path_index].catch_point_id;//抓取点编号
			//path.pose = path_planning_struct->path_point[curr_path_index].data();  //轨迹点数据
			S32 axis_index = vec_path_data[curr_path_index].axis_id;			   //轴编号
			float theta[6][30] = { -0.850283046871025 ,-1.13879516544585 ,-1.14828866146744 ,-1.68011665016359 ,-1.70549877077478 ,-2.73045272764916 ,-3.02051602765832, -3.86532778466132 ,-4.40848689748384 ,-4.43049108843667 ,-4.04502794141089, -3.52919002126223, -2.67487618832766, -2.79613505827441 ,-2.84937927350427, -2.93131633925437 ,-3.31561951756996, -3.83895727405696, -4.57863735482207, -5.58642005396746, -6.93482569909875 ,-8.72196602117448 ,-11.0784714302369, -14.1769232686237 ,-18.2444286537696 ,-23.5792307607967, -30.5725567315059 ,-39.7372967604476, -51.7456128637177 ,-67.4782329366585,
				-0.850283046871025 ,-1.13879516544585 ,-1.14828866146744 ,-1.68011665016359 ,-1.70549877077478 ,-2.73045272764916 ,-3.02051602765832, -3.86532778466132 ,-4.40848689748384 ,-4.43049108843667 ,-4.04502794141089, -3.52919002126223, -2.67487618832766, -2.79613505827441 ,-2.84937927350427, -2.93131633925437 ,-3.31561951756996, -3.83895727405696, -4.57863735482207, -5.58642005396746, -6.93482569909875 ,-8.72196602117448 ,-11.0784714302369, -14.1769232686237 ,-18.2444286537696 ,-23.5792307607967, -30.5725567315059 ,-39.7372967604476, -51.7456128637177 ,-67.4782329366585 ,
				-0.850283046871025 ,-1.13879516544585 ,-1.14828866146744 ,-1.68011665016359 ,-1.70549877077478 ,-2.73045272764916 ,-3.02051602765832, -3.86532778466132 ,-4.40848689748384 ,-4.43049108843667 ,-4.04502794141089, -3.52919002126223, -2.67487618832766, -2.79613505827441 ,-2.84937927350427, -2.93131633925437 ,-3.31561951756996, -3.83895727405696, -4.57863735482207, -5.58642005396746, -6.93482569909875 ,-8.72196602117448 ,-11.0784714302369, -14.1769232686237 ,-18.2444286537696 ,-23.5792307607967, -30.5725567315059 ,-39.7372967604476, -51.7456128637177 ,-67.4782329366585 ,
				-0.850283046871025 ,-1.13879516544585 ,-1.14828866146744 ,-1.68011665016359 ,-1.70549877077478 ,-2.73045272764916 ,-3.02051602765832, -3.86532778466132 ,-4.40848689748384 ,-4.43049108843667 ,-4.04502794141089, -3.52919002126223, -2.67487618832766, -2.79613505827441 ,-2.84937927350427, -2.93131633925437 ,-3.31561951756996, -3.83895727405696, -4.57863735482207, -5.58642005396746, -6.93482569909875 ,-8.72196602117448 ,-11.0784714302369, -14.1769232686237 ,-18.2444286537696 ,-23.5792307607967, -30.5725567315059 ,-39.7372967604476, -51.7456128637177 ,-67.4782329366585 ,
				-0.850283046871025 ,-1.13879516544585 ,-1.14828866146744 ,-1.68011665016359 ,-1.70549877077478 ,-2.73045272764916 ,-3.02051602765832, -3.86532778466132 ,-4.40848689748384 ,-4.43049108843667 ,-4.04502794141089, -3.52919002126223, -2.67487618832766, -2.79613505827441 ,-2.84937927350427, -2.93131633925437 ,-3.31561951756996, -3.83895727405696, -4.57863735482207, -5.58642005396746, -6.93482569909875 ,-8.72196602117448 ,-11.0784714302369, -14.1769232686237 ,-18.2444286537696 ,-23.5792307607967, -30.5725567315059 ,-39.7372967604476, -51.7456128637177 ,-67.4782329366585 ,
				-0.850283046871025 ,-1.13879516544585 ,-1.14828866146744 ,-1.68011665016359 ,-1.70549877077478 ,-2.73045272764916 ,-3.02051602765832, -3.86532778466132 ,-4.40848689748384 ,-4.43049108843667 ,-4.04502794141089, -3.52919002126223, -2.67487618832766, -2.79613505827441 ,-2.84937927350427, -2.93131633925437 ,-3.31561951756996, -3.83895727405696, -4.57863735482207, -5.58642005396746, -6.93482569909875 ,-8.72196602117448 ,-11.0784714302369, -14.1769232686237 ,-18.2444286537696 ,-23.5792307607967, -30.5725567315059 ,-39.7372967604476, -51.7456128637177 ,-67.4782329366585 };
			float direct[6] = { 1,1,1,1,1,1 };
			float delta[3];
			std::vector<HcRobotPose> interpolation_point;
			HcIKProcessParams ik_process_params; //逆解模块动态参数
			ik_process_params.pos = path_planning_struct->path_point[curr_path_index][0];
			S32 n_ret = HcIK::hcIKProcessing(
				path_planning_struct->ik_ppcf_handle, &ik_process_params);
			bool is_joint_ok;
			if (HANCHINE_S_OK == n_ret)
			{
				for (int i = 0; i < 6; i++)
				{
					path_planning_struct->path_point[curr_path_index][0].joint[i] = ik_process_params.result.data[i];
				}
				goalpoint = path_planning_struct->path_point[curr_path_index][0];
				is_joint_ok = getFreeCollisionPath(path_planning_struct->DH, interpolation_point, startpoint, goalpoint, theta, direct, path_planning_struct->joint_max_min.data);
				if (!is_joint_ok)
				{
					path.path_status = EnumHcJointStuck;
					path.error_point = path_planning_struct->path_point[curr_path_index][0];
					break;
				}
				for (int adjustnum = 0; adjustnum < 20; adjustnum++)
				{
					int collision_num = 0;
					for (curr_point_index = interpolation_point.size() - 1; curr_point_index > -1; curr_point_index--)
					{
						S32 is_collision = 0;
						Eigen::Matrix4f axis_pose = poseToRT(
							interpolation_point[curr_point_index]);
						HcCollisionDetectionProcessParams collision_process_params;
						collision_process_params.axis_pose = axis_pose.data();
						collision_process_params.axis_index = axis_index;
						collision_process_params.all_point_num = path_planning_struct->detected_collision_point_num;
						collision_process_params.inner_point_num = path_planning_struct->target_object_collision_num;
						collision_process_params.workpiece_pose = vec_path_data[curr_path_index].workpiece_pose.data();
						collision_process_params.inner_point_dis = path_planning_struct->inner_point_dis;
						collision_process_params.collision_type =
							EnumHcAxisAndEenvir | EnumHcAxisAndScan | EnumHcAxisAndWorkpiece;
						if (path_planning_struct->is_detect_box)
						{
							collision_process_params.collision_type |= EnumHcAxisAndBox;
						}
						HcCollisionDetection::hcCollisionDetectionProcessing(
							path_planning_struct->collision_ppcf_handle, &collision_process_params);
						is_collision = collision_process_params.result;
						if (is_collision == 1)
						{
							if (curr_point_index == interpolation_point.size() - 1)
							{
								path.path_status = EnumHcCollision;
								path.pose = interpolation_point.data();
								path.size = interpolation_point.size();
								adjustnum = 100;
								break;
							}
							pathAdjust(interpolation_point[curr_point_index], path_planning_struct->DH, theta, collision_process_params.distance, curr_point_index, direct);
							collision_num++;
						}
					}
					if ((collision_num == 0)& (path.path_status == EnumHcPathOK))
					{
						path.path_status = EnumHcPathOK;
						path.pose = interpolation_point.data();
						path.size = interpolation_point.size();
						break;
					}
					if (adjustnum == 4)
					{
						path.path_status = EnumHcCollision;
						path.pose = interpolation_point.data();
						path.size = interpolation_point.size();
						break;
					}
				}

			}

			if (EnumHcPathOK == path.path_status)
			{
				path.path_status = EnumHcPathOK;
				path_planning_struct->vec_all_path.push_back(path);
				path_planning_struct->vec_ok_path.push_back(path);
				path_ok_num++;
				workpiece_index[path.workpiece_index] = 1;
			}
			else
			{
				path_planning_struct->vec_all_path.push_back(path);
				if (0 == workpiece_index[path.workpiece_index])
				{
					workpiece_index[path.workpiece_index] = -1;
				}
			}
			for (int i = 0; i < 500; i++)
			{
				if (workpiece_index[i] < 0)
				{
					path_planning_struct->blacklist.push_back(i);
				}
			}
			ppcf_process_params->blacklist_data = path_planning_struct->blacklist.data();
			ppcf_process_params->blacklist_size = path_planning_struct->blacklist.size();
           }
			if (EnumHcWorking == path_planning_struct->running_model)
			{
				//生成轨迹点关节角度
				U32 path_size = path_planning_struct->vec_ok_path.size();
				for (U32 i = 0; i < path_size; i++)
				{
					for (U32 j = 0; j < point_size; j++)
					{
						HcIKProcessParams ik_process_params; //逆解模块动态参数
						ik_process_params.pos = path_planning_struct->vec_ok_path[i].pose[j];
						S32 n_ret = HcIK::hcIKProcessing(
							path_planning_struct->ik_ppcf_handle, &ik_process_params);
						HcJoint ik_result = ik_process_params.result;
						getJointMat(path_planning_struct->DH, ik_result,
							path_planning_struct->robot_mat,
							path_planning_struct->tool_long_axiz,
							path_planning_struct->tool_rotation_axiz,
							path_planning_struct->vec_ok_path[i].seven_axis_angle,
							path_planning_struct->is_seven_axiz);
					}
				}
				for (int i = 0; i < path_size; i++)
				{
					path_planning_struct->vec_ok_path[i].robot_joint =
						path_planning_struct->robot_mat[i * 8 * point_size].data();
				}
				ppcf_process_params->path_list.path_data = path_planning_struct->vec_ok_path.data();
				ppcf_process_params->path_list.size = path_planning_struct->vec_ok_path.size();
			}
			if (EnumHcTesting == path_planning_struct->running_model)
			{
				//生成轨迹点关节角度
				U32 path_size = path_planning_struct->vec_all_path.size();
				for (U32 i = 0; i < path_size; i++)
				{
					for (U32 j = 0; j < point_size; j=j+9)
					{
						HcIKProcessParams ik_process_params; //逆解模块动态参数
						ik_process_params.pos = path_planning_struct->vec_all_path[i].pose[j];
						S32 n_ret = HcIK::hcIKProcessing(
							path_planning_struct->ik_ppcf_handle, &ik_process_params);
						HcJoint ik_result;
						if (HANCHINE_S_OK == n_ret)
						{
							ik_result = ik_process_params.result;
						}
						else
						{
							ik_result.joint1 = 0;
							ik_result.joint2 = 0;
							ik_result.joint3 = 0;
							ik_result.joint4 = 0;
							ik_result.joint5 = 0;
							ik_result.joint6 = 0;
						}

						getJointMat(path_planning_struct->DH, ik_result,
							path_planning_struct->robot_mat,
							path_planning_struct->tool_long_axiz,
							path_planning_struct->tool_rotation_axiz,
							path_planning_struct->vec_all_path[i].seven_axis_angle,
							path_planning_struct->is_seven_axiz);
					}
				}
				for (int i = 0; i < path_size; i++)
				{
					path_planning_struct->vec_all_path[i].robot_joint =
						path_planning_struct->robot_mat[i * 8 * (point_size-35)].data();
				}
				ppcf_process_params->path_list.path_data = path_planning_struct->vec_all_path.data();
				ppcf_process_params->path_list.size = path_planning_struct->vec_all_path.size();
			}
		return process_state;
	}
	//五点法
	if (EnumHcFivePoint == path_planning_struct->path_type)
	{
		for (curr_path_index = 0; curr_path_index < path_planning_struct->path_point.size(); curr_path_index++)
		{

			//插补点集
			HcInterpolationProcessParams interpolation_process_params; //插补模块动态参数
			std::vector<HcRobotPose> interpolation_point;
			for (curr_point_index = 2; curr_point_index < 4; curr_point_index++)
			{
				interpolation_process_params.initial_position = path_planning_struct->path_point[curr_path_index][curr_point_index];
				interpolation_process_params.goal_position = path_planning_struct->path_point[curr_path_index][curr_point_index + 1];
				interpolation_process_params.num = path_planning_struct->interpolation_point_num[curr_point_index - 2];
				S32 n_ret = HcInterpolation::hcInterpolationProcessing(
					path_planning_struct->interpolation_ppcf_handle, &interpolation_process_params);
				if (HANCHINE_S_OK == n_ret)
				{
					std::vector<HcRobotPose> tmp =
						*(std::vector<HcRobotPose>*)interpolation_process_params.output;
					for (S32 i = 0; i < tmp.size(); i++)
					{
						interpolation_point.push_back(tmp[i]);
					}
				}
			}
			interpolation_path_point.push_back(interpolation_point);
		}

		//std::cout << "generate interpolation_path_point time:" << double(endTime - startTime) << "ms" << std::endl;

		//点数

		if (EnumHcFivePoint == path_planning_struct->path_type)
		{
			point_size = 5;
		}

		//轨迹判断
		startTime = clock();
		curr_path_index = 0;
		S32 path_ok_num = 0;
		while (curr_path_index < interpolation_path_point.size() &&
			path_ok_num < path_planning_struct->path_num_max &&
			curr_path_index < 500)
		{
			HcPath path;
			memset(&path, 0, sizeof(HcPath));
			path.workpiece_index = vec_path_data[curr_path_index].workpiece_id;    //工件编号
			path.catch_point_index = vec_path_data[curr_path_index].catch_point_id;//抓取点编号
			path.pose = path_planning_struct->path_point[curr_path_index].data();  //轨迹点数据
			S32 axis_index = vec_path_data[curr_path_index].axis_id;			   //轴编号
			if (path_planning_struct->is_seven_axiz)
			{
				path.seven_axis_angle = path_planning_struct->vec_axis_angle[axis_index];
			}
			path.size = point_size;

			path.path_status = EnumHcPathOK;
			//判断关节卡死
			for (curr_point_index = 0; curr_point_index < interpolation_path_point[curr_path_index].size(); curr_point_index++)
			{
				HcIKProcessParams ik_process_params; //逆解模块动态参数
				ik_process_params.pos = interpolation_path_point[curr_path_index][curr_point_index];
				S32 n_ret = HcIK::hcIKProcessing(
					path_planning_struct->ik_ppcf_handle, &ik_process_params);
				if (HANCHINE_S_OK == n_ret)
				{
					HcJoint ik_result = ik_process_params.result;
					//if (abs(ik_result.data[4]) < 0.1)
					//{
						//path.path_status = EnumHcJointStuck;
					//}
					for (S32 i = 0; i < 6; i++)
					{
						if (ik_result.data[i] < path_planning_struct->joint_max_min.data[2 * i] ||
							ik_result.data[i] > path_planning_struct->joint_max_min.data[2 * i + 1])
						{
							//发生关节卡死。。。
							path.path_status = EnumHcJointStuck;
						}
					}
				}
				else
				{
					path.path_status = EnumHcJointStuck;
				}
				if (EnumHcJointStuck == path.path_status)
				{
					path.error_point = interpolation_path_point[curr_path_index][curr_point_index];
					break;
				}
			}
			//判断是否碰撞
			if (EnumHcPathOK == path.path_status)
			{
				for (curr_point_index = 0; curr_point_index < interpolation_path_point[curr_path_index].size(); curr_point_index++)
				{
					S32 is_collision = 0;
					Eigen::Matrix4f axis_pose = poseToRT(
						interpolation_path_point[curr_path_index][curr_point_index]);
					//F32* tmp_data = new F32[3 * path_planning_struct->concat_axis_size];
					//memcpy(tmp_data, path_planning_struct->concat_axis_data,
					//	3 * path_planning_struct->concat_axis_size * sizeof(F32));
					////transPose(tmp_data, path_planning_struct->concat_axis_size, axis_pose.data());
					//saveMatToPCD(tmp_data,
					//	path_planning_struct->concat_axis_size,
					//	"./concat_axis.pcd");
					//delete[] tmp_data;

					HcCollisionDetectionProcessParams collision_process_params;
					collision_process_params.axis_pose = axis_pose.data();
					collision_process_params.axis_index = axis_index;
					collision_process_params.all_point_num = path_planning_struct->detected_collision_point_num;
					collision_process_params.inner_point_num = path_planning_struct->target_object_collision_num;
					collision_process_params.workpiece_pose = vec_path_data[curr_path_index].workpiece_pose.data();
					collision_process_params.inner_point_dis = path_planning_struct->inner_point_dis;
					collision_process_params.collision_type =
						EnumHcAxisAndEenvir | EnumHcAxisAndScan | EnumHcAxisAndWorkpiece;
					if (path_planning_struct->is_detect_box)
					{
						collision_process_params.collision_type |= EnumHcAxisAndBox;
					}
					HcCollisionDetection::hcCollisionDetectionProcessing(
						path_planning_struct->collision_ppcf_handle, &collision_process_params);
					is_collision = collision_process_params.result;
					if (is_collision == 1)
					{
						//发生碰撞。。。
						path.path_status = EnumHcCollision;
						path.error_point = interpolation_path_point[curr_path_index][curr_point_index];
						break;
					}
				}
			}

			if (EnumHcPathOK == path.path_status)
			{
				path.path_status = EnumHcPathOK;
				path_planning_struct->vec_all_path.push_back(path);
				path_planning_struct->vec_ok_path.push_back(path);
				curr_path_index++;
				path_ok_num++;
				workpiece_index[path.workpiece_index] = 1;
			}
			else
			{
				path_planning_struct->vec_all_path.push_back(path);
				curr_path_index++;
				if (0 == workpiece_index[path.workpiece_index])
				{
					workpiece_index[path.workpiece_index] = -1;
				}
			}
		}
		endTime = clock();
		//std::cout << "path filter time:" << double(endTime - startTime) << "ms" << std::endl;

		for (int i = 0; i < 500; i++)
		{
			if (workpiece_index[i] < 0)
			{
				path_planning_struct->blacklist.push_back(i);
			}
		}
		ppcf_process_params->blacklist_data = path_planning_struct->blacklist.data();
		ppcf_process_params->blacklist_size = path_planning_struct->blacklist.size();

		if (EnumHcWorking == path_planning_struct->running_model)
		{
			//生成轨迹点关节角度
			U32 path_size = path_planning_struct->vec_ok_path.size();
			for (U32 i = 0; i < path_size; i++)
			{
				for (U32 j = 0; j < point_size; j++)
				{
					HcIKProcessParams ik_process_params; //逆解模块动态参数
					ik_process_params.pos = path_planning_struct->vec_ok_path[i].pose[j];
					S32 n_ret = HcIK::hcIKProcessing(
						path_planning_struct->ik_ppcf_handle, &ik_process_params);
					HcJoint ik_result = ik_process_params.result;
					getJointMat(path_planning_struct->DH, ik_result,
						path_planning_struct->robot_mat,
						path_planning_struct->tool_long_axiz,
						path_planning_struct->tool_rotation_axiz,
						path_planning_struct->vec_ok_path[i].seven_axis_angle,
						path_planning_struct->is_seven_axiz);
				}
			}
			for (int i = 0; i < path_size; i++)
			{
				path_planning_struct->vec_ok_path[i].robot_joint =
					path_planning_struct->robot_mat[i * 8 * point_size].data();
			}
			ppcf_process_params->path_list.path_data = path_planning_struct->vec_ok_path.data();
			ppcf_process_params->path_list.size = path_planning_struct->vec_ok_path.size();
		}
		if (EnumHcTesting == path_planning_struct->running_model)
		{
			//生成轨迹点关节角度
			U32 path_size = path_planning_struct->vec_all_path.size();
			for (U32 i = 0; i < path_size; i++)
			{
				for (U32 j = 0; j < point_size; j++)
				{
					HcIKProcessParams ik_process_params; //逆解模块动态参数
					ik_process_params.pos = path_planning_struct->vec_all_path[i].pose[j];
					S32 n_ret = HcIK::hcIKProcessing(
						path_planning_struct->ik_ppcf_handle, &ik_process_params);
					HcJoint ik_result;
					if (HANCHINE_S_OK == n_ret)
					{
						ik_result = ik_process_params.result;
					}
					else
					{
						ik_result.joint1 = 0;
						ik_result.joint2 = 0;
						ik_result.joint3 = 0;
						ik_result.joint4 = 0;
						ik_result.joint5 = 0;
						ik_result.joint6 = 0;
					}

					getJointMat(path_planning_struct->DH, ik_result,
						path_planning_struct->robot_mat,
						path_planning_struct->tool_long_axiz,
						path_planning_struct->tool_rotation_axiz,
						path_planning_struct->vec_all_path[i].seven_axis_angle,
						path_planning_struct->is_seven_axiz);
				}
			}
			for (int i = 0; i < path_size; i++)
			{
				path_planning_struct->vec_all_path[i].robot_joint =
					path_planning_struct->robot_mat[i * 8 * point_size].data();
			}
			ppcf_process_params->path_list.path_data = path_planning_struct->vec_all_path.data();
			ppcf_process_params->path_list.size = path_planning_struct->vec_all_path.size();
		}

		return process_state;
	}
}


PATH_PLANNING_API S32 HcPathPlanning::hcPathPlanningRelease(IN void *ppcf_handle)
{
	if (NULL == ppcf_handle)
	{
		return HC_PATH_PLANNING_E_PARAM_NULL;
	}
	HcPathPlanningStruct *path_planning_struct = (HcPathPlanningStruct *)ppcf_handle;

	delete[] path_planning_struct->box_mesh_data;
	delete[] path_planning_struct->envir_mesh_data;
	delete[] path_planning_struct->long_axiz_mesh_data;
	delete[] path_planning_struct->short_axiz_mesh_data;
	delete[] path_planning_struct->concat_axis_data;
	delete[] path_planning_struct->workpiece_mesh_data;

	path_planning_struct->vec_all_path.swap(std::vector<HcPath>());
	path_planning_struct->vec_ok_path.swap(std::vector<HcPath>());
	path_planning_struct->path_point.swap(std::vector<std::vector<HcRobotPose>>());
	path_planning_struct->interpolation_path_point.swap(std::vector<std::vector<HcRobotPose>>());
	path_planning_struct->vec_axis_pose.swap(std::vector<Eigen::Matrix4f>());
	path_planning_struct->vec_axis_angle.swap(std::vector<F32>());
	path_planning_struct->vec_TCP.swap(std::vector<Eigen::Matrix4f>());
	path_planning_struct->vec_catch_point.swap(std::vector<HcCatchPoint>());
	path_planning_struct->blacklist.swap(std::vector<U32>());
	path_planning_struct->robot_mat.swap(std::vector<Eigen::Matrix4f>());

	//释放模块句柄
	HcCollisionDetection::hcCollisionDetectionRelease(path_planning_struct->collision_ppcf_handle);
	HcInterpolation::hcInterpolationRelease(path_planning_struct->interpolation_ppcf_handle);
	HcIK::hcIKRelease(path_planning_struct->ik_ppcf_handle);
	HcCatchPointSort::hcCatchPointSortRelease(path_planning_struct->sort_ppdf_handle);

	free(ppcf_handle);
	ppcf_handle = NULL;

	return HANCHINE_S_OK;
}