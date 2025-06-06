﻿using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using jkType;

namespace jakaApi
{
	public delegate void CallBackFuncType(int error_code);
	
	public class jakaAPI
	{
		#region general

		[DllImport("jakaAPI.dll", EntryPoint = "create_handler", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int create_handler(char[] ip, ref int handle, bool use_grpc = false);

		[DllImport("jakaAPI.dll", EntryPoint = "create_handler", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int create_handler([MarshalAs(UnmanagedType.LPStr)] string ip, ref int handle, bool use_grpc = false);

		[DllImport("jakaAPI.dll", EntryPoint = "destory_handler", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int destory_handler(ref int handle);

		[DllImport("jakaAPI.dll", EntryPoint = "power_on", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int power_on(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "power_off", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int power_off(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "shut_down", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int shut_down(ref int handle);

		[DllImport("jakaAPI.dll", EntryPoint = "enable_robot", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int enable_robot(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "disable_robot", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int disable_robot(ref int handle);	

		[DllImport("jakaAPI.dll", EntryPoint = "get_dh_param", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_dh_param(ref int i,ref JKTYPE.DHParam offset);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_installation_angle", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_installation_angle(ref int i, double angleX, double angleZ);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_installation_angle", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_installation_angle(ref int i, ref JKTYPE.Quaternion quat, ref JKTYPE.Rpy appang);
						
		[DllImport("jakaAPI.dll", EntryPoint = "get_robot_state", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_robot_state(ref int handle, ref JKTYPE.RobotState state);

		[DllImport("jakaAPI.dll", EntryPoint = "get_robot_status", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_robot_status(ref int i, ref JKTYPE.RobotStatus status);

		[DllImport("jakaAPI.dll", EntryPoint = "get_robot_status_simple", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_robot_status_simple(ref int i, ref JKTYPE.RobotStatus_simple status);

		

		#endregion

		#region SDK support
		[DllImport("jakaAPI.dll", EntryPoint = "set_error_handler", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_error_handler(ref int i, CallBackFuncType func);

		[DllImport("jakaAPI.dll", EntryPoint = "get_sdk_version", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_sdk_version(ref int i, StringBuilder version, int size);

		[DllImport("jakaAPI.dll", EntryPoint = "set_network_exception_handle", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_network_exception_handle(ref int i, float millisecond, JKTYPE.ProcessType mnt);

		[DllImport("jakaAPI.dll", EntryPoint = "get_controller_ip", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_controller_ip(char[] controller_name, StringBuilder ip);

		[DllImport("jakaAPI.dll", EntryPoint = "get_controller_ip", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_controller_ip([MarshalAs(UnmanagedType.LPStr)] string controller_name, StringBuilder ip);

		[DllImport("jakaAPI.dll", EntryPoint = "set_errorcode_file_path", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_errorcode_file_path(ref int i, StringBuilder path);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_last_error", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_last_error(ref int i, ref JKTYPE.ErrorCode code);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_debug_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_debug_mode(ref int i, bool mode);

		[DllImport("jakaAPI.dll", EntryPoint = "set_SDK_filepath", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_SDK_filepath(ref int i, ref char[] filepath);

		[DllImport("jakaAPI.dll", EntryPoint = "set_SDK_filepath", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_SDK_filepath(ref int i, [MarshalAs(UnmanagedType.LPStr)] string filepath);

		[DllImport("jakaAPI.dll", EntryPoint = "get_SDK_filepath", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_SDK_filepath(char[] path, int size);

		[DllImport("jakaAPI.dll", EntryPoint = "get_SDK_filepath", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_SDK_filepath(StringBuilder path, int size);



		#endregion

		#region motion

		[DllImport("jakaAPI.dll", EntryPoint = "jog", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int jog(ref int handle, int aj_num, JKTYPE.MoveMode move_mode, JKTYPE.CoordType coord_type, double vel_cmd, double pos_cmd);
		
		[DllImport("jakaAPI.dll", EntryPoint = "jog_stop", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int jog_stop(ref int handle, int num);
		
		[DllImport("jakaAPI.dll", EntryPoint = "joint_move", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int joint_move(ref int handle, ref JKTYPE.JointValue joint_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed);

		[DllImport("jakaAPI.dll", EntryPoint = "joint_move_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int joint_move_extend(ref int i, ref JKTYPE.JointValue joint_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed, double acc, double tol, ref JKTYPE.OptionalCond option_cond);

		[DllImport("jakaAPI.dll", EntryPoint = "linear_move", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int linear_move(ref int handle, ref JKTYPE.CartesianPose end_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed);

		[DllImport("jakaAPI.dll", EntryPoint = "linear_move_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int linear_move_extend(ref int i, ref JKTYPE.CartesianPose cart_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed, double acc, double tol, ref JKTYPE.OptionalCond option_cond);
       
	    [DllImport("jakaAPI.dll", EntryPoint = "linear_move_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
        public static extern int linear_move_extend_ori(ref int i, ref JKTYPE.CartesianPose cart_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed, double acc, double tol, ref JKTYPE.OptionalCond option_cond, double ori_vel, double ori_acc);
				
		[DllImport("jakaAPI.dll", EntryPoint = "circular_move", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int circular_move(ref int i, ref JKTYPE.CartesianPose end_pos, ref JKTYPE.CartesianPose mid_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed, double acc, double tol, ref JKTYPE.OptionalCond option_cond);
		
		[DllImport("jakaAPI.dll", EntryPoint = "circular_move_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int circular_move_extend(ref int i, ref JKTYPE.CartesianPose end_pos, ref JKTYPE.CartesianPose mid_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed, double acc, double tol, ref JKTYPE.OptionalCond option_cond, double circle_cnt);
		
		[DllImport("jakaAPI.dll", EntryPoint = "circular_move_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int circular_move_extend_mode(ref int i, ref JKTYPE.CartesianPose end_pos, ref JKTYPE.CartesianPose mid_pos, JKTYPE.MoveMode move_mode, bool is_block, double speed, double acc, double tol, ref JKTYPE.OptionalCond option_cond, double circle_cnt, int circle_mode);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_rapidrate", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_rapidrate(ref int handle, double rapid_rate);

		[DllImport("jakaAPI.dll", EntryPoint = "get_rapidrate", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_rapidrate(ref int handle, ref double rapid_rate);

		[DllImport("jakaAPI.dll", EntryPoint = "set_tool_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tool_data(ref int handle, int id, ref JKTYPE.CartesianPose tcp, char[] name);

		[DllImport("jakaAPI.dll", EntryPoint = "set_tool_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tool_data(ref int handle, int id, ref JKTYPE.CartesianPose tcp, [MarshalAs(UnmanagedType.LPStr)] string name);

		[DllImport("jakaAPI.dll", EntryPoint = "get_tool_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tool_data(ref int handle, int id, ref JKTYPE.CartesianPose tcp);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_tool_id", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tool_id(ref int handle, int id);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_tool_id", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tool_id(ref int handle, ref int id);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_user_frame_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_user_frame_data(ref int handle, int id, ref JKTYPE.CartesianPose user_frame, char[] name);

		[DllImport("jakaAPI.dll", EntryPoint = "set_user_frame_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_user_frame_data(ref int handle, int id, ref JKTYPE.CartesianPose user_frame, [MarshalAs(UnmanagedType.LPStr)] string name);

		[DllImport("jakaAPI.dll", EntryPoint = "get_user_frame_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_user_frame_data(ref int handle, int id, ref JKTYPE.CartesianPose user_frame);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_user_frame_id", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_user_frame_id(ref int handle, int id);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_user_frame_id", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_user_frame_id(ref int handle, ref int id);
		
		[DllImport("jakaAPI.dll", EntryPoint = "drag_mode_enable", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int drag_mode_enable(ref int handle, bool enable);
		
		[DllImport("jakaAPI.dll", EntryPoint = "is_in_drag_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_in_drag_mode(ref int handle, ref bool in_drag);

		[DllImport("jakaAPI.dll", EntryPoint = "get_tcp_position", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tcp_position(ref int handle, ref JKTYPE.CartesianPose tcp_position);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_joint_position", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_joint_position(ref int handle, ref JKTYPE.JointValue joint_position);

		[DllImport("jakaAPI.dll", EntryPoint = "get_actual_tcp_position", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_actual_tcp_position(ref int handle, ref JKTYPE.CartesianPose tcp_position);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_actual_joint_position", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_actual_joint_position(ref int handle, ref JKTYPE.JointValue joint_position);

		[DllImport("jakaAPI.dll", EntryPoint = "is_in_estop", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_in_estop(ref int handle, ref bool estop);
		
		[DllImport("jakaAPI.dll", EntryPoint = "is_on_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_on_limit(ref int handle, ref bool on_limit);
		
		[DllImport("jakaAPI.dll", EntryPoint = "is_in_pos", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_in_pos(ref int handle, ref bool in_pos);

		[DllImport("jakaAPI.dll", EntryPoint = "set_motion_planner", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_motion_planner(ref int i, int type);

		[DllImport("jakaAPI.dll", EntryPoint = "motion_abort", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int motion_abort(ref int handle);

		[DllImport("jakaAPI.dll", EntryPoint = "get_motion_status", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_motion_status(ref int handle, JKTYPE.MotionStatus status);

		[DllImport("jakaAPI.dll", EntryPoint = "set_payload", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_payload(ref int handle, ref JKTYPE.PayLoad payload);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_payload", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_payload(ref int handle, ref JKTYPE.PayLoad payload);

		[DllImport("jakaAPI.dll", EntryPoint = "set_drag_friction_compensation_gain", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_drag_friction_compensation_gain(ref int handle, int joint, int gain);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_drag_friction_compensation_gain", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_drag_friction_compensation_gain(ref int handle, ref JKTYPE.DragFrictionCompensationGainList gainList);


		#endregion

		#region IO

		[DllImport("jakaAPI.dll", EntryPoint = "set_digital_output", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_digital_output(ref int handle, JKTYPE.IOType type, int index, bool value);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_analog_output", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_analog_output(ref int handle, JKTYPE.IOType type, int index, float value);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_digital_input", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_digital_input(ref int handle, JKTYPE.IOType type, int index, ref bool result);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_digital_output", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_digital_output(ref int handle, JKTYPE.IOType type, int index, ref bool result);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_analog_input", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_analog_input(ref int handle, JKTYPE.IOType type, int index, ref float result);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_analog_output", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_analog_output(ref int handle, JKTYPE.IOType type, int index, ref float result);
		
		[DllImport("jakaAPI.dll", EntryPoint = "is_extio_running", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_extio_running(ref int handle, ref bool is_running);

		[DllImport("jakaAPI.dll", EntryPoint = "set_motion_digital_output", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_motion_digital_output(ref int handle, JKTYPE.IOType type, int index, bool value);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_motion_analog_output", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_motion_analog_output(ref int handle, JKTYPE.IOType type, int index, float value);

		
		#endregion

		#region  tio

		[DllImport("jakaAPI.dll", EntryPoint = "set_tio_vout_param", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tio_vout_param(ref int i, int vout_enable,int vout_vol);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_tio_vout_param", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tio_vout_param(ref int i,ref int vout_enable,ref int vout_vol);
		
		[DllImport("jakaAPI.dll", EntryPoint = "add_tio_rs_signal", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int add_tio_rs_signal(ref int i, JKTYPE.SignInfo sign_info);

		[DllImport("jakaAPI.dll", EntryPoint = "del_tio_rs_signal", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int del_tio_rs_signal(ref int i, char[] sig_name);

		[DllImport("jakaAPI.dll", EntryPoint = "del_tio_rs_signal", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int del_tio_rs_signal(ref int i, [MarshalAs(UnmanagedType.LPStr)] string sig_name);

		[DllImport("jakaAPI.dll", EntryPoint = "send_tio_rs_command", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int send_tio_rs_command(ref int i, int chn_id, byte[] data, int buffsize);

		[DllImport("jakaAPI.dll", EntryPoint = "get_rs485_signal_info", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_rs485_signal_info(ref int i, [In, Out] JKTYPE.SignInfo[] sign_info, ref int size);

		[DllImport("jakaAPI.dll", EntryPoint = "set_tio_pin_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tio_pin_mode(ref int i, int pin_type, int pin_mode);

		[DllImport("jakaAPI.dll", EntryPoint = "get_tio_pin_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tio_pin_mode(ref int i, int pin_type, ref int pin_mode);

		[DllImport("jakaAPI.dll", EntryPoint = "set_rs485_chn_comm", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_rs485_chn_comm(ref int i, JKTYPE.ModRtuComm mod_rtu_com);

		[DllImport("jakaAPI.dll", EntryPoint = "get_rs485_chn_comm", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_rs485_chn_comm(ref int i, ref JKTYPE.ModRtuComm mod_rtu_com);

		[DllImport("jakaAPI.dll", EntryPoint = "set_rs485_chn_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_rs485_chn_mode(ref int i, int chn_id, int chn_mode);

		[DllImport("jakaAPI.dll", EntryPoint = "get_rs485_chn_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_rs485_chn_mode(ref int handle, int chn_id, ref int chn_mode);

		[DllImport("jakaAPI.dll", EntryPoint = "tio_sensor_calib", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int tio_sensor_calib(ref int handle, int type);

		[DllImport("jakaAPI.dll", EntryPoint = "update_tio_rs_signal", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int update_tio_rs_signal(ref int i, JKTYPE.SignInfo_simple sign_info);

		#endregion

		#region  servoJP

		[DllImport("jakaAPI.dll", EntryPoint = "servo_move_enable", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_move_enable(ref int handle, bool enable);

		[DllImport("jakaAPI.dll", EntryPoint = "is_in_servomove", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_in_servomove(ref int handle, bool in_servo);
		
		[DllImport("jakaAPI.dll", EntryPoint = "servo_j", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_j(ref int handle, ref JKTYPE.JointValue joint_pos, JKTYPE.MoveMode move_mode, int step_num);
		
		[DllImport("jakaAPI.dll", EntryPoint = "servo_p", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_p(ref int handle, ref JKTYPE.CartesianPose cartesian_pose, JKTYPE.MoveMode move_mode, int step_num);

		[DllImport("jakaAPI.dll", EntryPoint = "edg_servo_j", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int edg_servo_j(ref int handle, ref JKTYPE.JointValue joint_pos, JKTYPE.MoveMode move_mode, int step_num, byte robot_index);

		[DllImport("jakaAPI.dll", EntryPoint = "edg_servo_p", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int edg_servo_p(ref int handle, ref JKTYPE.CartesianPose cartesian_pose, JKTYPE.MoveMode move_mode, int step_num, byte robot_index);

		[DllImport("jakaAPI.dll", EntryPoint = "edg_init", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int edg_init(ref int handle, ref bool en, char[] edg_stat_ip);

		[DllImport("jakaAPI.dll", EntryPoint = "edg_get_stat", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int edg_get_stat(ref int handle, ref JKTYPE.EDGState edg_state, byte robot_index);

		[DllImport("jakaAPI.dll", EntryPoint = "edg_stat_details", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int edg_stat_details(ref int handle, ulong[] details);



		[DllImport("jakaAPI.dll", EntryPoint = "servo_move_use_none_filter", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_move_use_none_filter(ref int i);
	
		[DllImport("jakaAPI.dll", EntryPoint = "servo_move_use_joint_LPF", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_move_use_joint_LPF(ref int i, double cutoffFreq);
		
		[DllImport("jakaAPI.dll", EntryPoint = "servo_move_use_joint_NLF", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_move_use_joint_NLF(ref int i, double max_vr, double max_ar, double max_jr);
		
		[DllImport("jakaAPI.dll", EntryPoint = "servo_move_use_carte_NLF", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_move_use_carte_NLF(ref int i, double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);
		
		[DllImport("jakaAPI.dll", EntryPoint = "servo_move_use_joint_MMF", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_move_use_joint_MMF(ref int i, int max_buf, double kp, double kv, double ka);
		
		[DllImport("jakaAPI.dll", EntryPoint = "servo_speed_foresight", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int servo_speed_foresight(ref int i, int max_buf, double kp);
		
		#endregion

		#region traj

		[DllImport("jakaAPI.dll", EntryPoint = "set_traj_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_traj_config(ref int i, ref JKTYPE.TrajTrackPara para);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_traj_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_traj_config(ref int i, ref JKTYPE.TrajTrackPara para);

		[DllImport("jakaAPI.dll", EntryPoint = "set_traj_sample_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_traj_sample_mode(ref int i, bool mode, char[] filename);

		[DllImport("jakaAPI.dll", EntryPoint = "set_traj_sample_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_traj_sample_mode(ref int i, bool mode, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport("jakaAPI.dll", EntryPoint = "get_traj_sample_status", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_traj_sample_status(ref int i, ref bool sample_statuse);

		[DllImport("jakaAPI.dll", EntryPoint = "get_exist_traj_file_name", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_exist_traj_file_name(ref int i, ref JKTYPE.MultStrStorType filename);

		[DllImport("jakaAPI.dll", EntryPoint = "rename_traj_file_name", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rename_traj_file_name(ref int i, ref char[] src, ref char[] dest);

		[DllImport("jakaAPI.dll", EntryPoint = "rename_traj_file_name", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rename_traj_file_name(ref int i, [MarshalAs(UnmanagedType.LPStr)] string src, [MarshalAs(UnmanagedType.LPStr)] string dest);

		[DllImport("jakaAPI.dll", EntryPoint = "remove_traj_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int remove_traj_file(ref int i, ref char[] filename);

		[DllImport("jakaAPI.dll", EntryPoint = "remove_traj_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int remove_traj_file(ref int i, [MarshalAs(UnmanagedType.LPStr)] string filename);

		[DllImport("jakaAPI.dll", EntryPoint = "generate_traj_exe_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int generate_traj_exe_file(ref int i, char[] filename);

		[DllImport("jakaAPI.dll", EntryPoint = "generate_traj_exe_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int generate_traj_exe_file(ref int i, [MarshalAs(UnmanagedType.LPStr)] string filename);

		#endregion

		#region program

		[DllImport("jakaAPI.dll", EntryPoint = "program_run", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int program_run(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "program_pause", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int program_pause(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "program_resume", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int program_resume(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "program_abort", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int program_abort(ref int handle);
		
		[DllImport("jakaAPI.dll", EntryPoint = "program_load", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int program_load(ref int handle, char[] file);

		[DllImport("jakaAPI.dll", EntryPoint = "program_load", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int program_load(ref int handle, [MarshalAs(UnmanagedType.LPStr)] string file);

		[DllImport("jakaAPI.dll", EntryPoint = "get_loaded_program", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_loaded_program(ref int handle, StringBuilder file, int len = 100);

		[DllImport("jakaAPI.dll", EntryPoint = "get_current_line", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_current_line(ref int handle, ref int curr_line);

		[DllImport("jakaAPI.dll", EntryPoint = "get_program_state", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_program_state(ref int handle, ref JKTYPE.ProgramState status);

		[DllImport("jakaAPI.dll", EntryPoint = "get_program_info", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_program_info(ref int handle, ref JKTYPE.ProgramInfo info);

		[DllImport("jakaAPI.dll", EntryPoint = "get_user_var", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_user_var(ref int handle, ref JKTYPE.UserVariableList vlist);

		[DllImport("jakaAPI.dll", EntryPoint = "set_user_var", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_user_var(ref int handle, ref JKTYPE.UserVariable v);
		
		#endregion

		#region FTP

		[DllImport("jakaAPI.dll", EntryPoint = "init_ftp_client", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int init_ftp_client(ref int i);
		
		[DllImport("jakaAPI.dll", EntryPoint = "close_ftp_client", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int close_ftp_client(ref int i);

		[DllImport("jakaAPI.dll", EntryPoint = "download_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int download_file(ref int i, char[] local, char[] remote, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "download_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int download_file(ref int i, [MarshalAs(UnmanagedType.LPStr)] string local, [MarshalAs(UnmanagedType.LPStr)] string remote, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "upload_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int upload_file(ref int i, char[] local, char[] remote, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "upload_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int upload_file(ref int i, [MarshalAs(UnmanagedType.LPStr)] string local, [MarshalAs(UnmanagedType.LPStr)] string remote, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "del_ftp_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int del_ftp_file(ref int i, char[] remote, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "del_ftp_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int del_ftp_file(ref int i, [MarshalAs(UnmanagedType.LPStr)] string remote, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "rename_ftp_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rename_ftp_file(ref int i, char[] remote, char[] des, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "rename_ftp_file", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rename_ftp_file(ref int i, [MarshalAs(UnmanagedType.LPStr)] string remote, [MarshalAs(UnmanagedType.LPStr)] string des, int opt);

		[DllImport("jakaAPI.dll", EntryPoint = "get_ftp_dir", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_ftp_dir(ref int i, char[] remote, int type, StringBuilder ret );

		[DllImport("jakaAPI.dll", EntryPoint = "get_ftp_dir", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_ftp_dir(ref int i, [MarshalAs(UnmanagedType.LPStr)] string remote, int type, StringBuilder ret );

		#endregion

		#region torque sensor and force base function			
		[DllImport("jakaAPI.dll", EntryPoint = "set_torsenosr_brand", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torsenosr_brand(ref int handle, int sensor_brand);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_torsenosr_brand", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torsenosr_brand(ref int handle, ref int sensor_brand);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_torque_sensor_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torque_sensor_mode(ref int handle, int sensor_mode);

		[DllImport("jakaAPI.dll", EntryPoint = "get_torque_sensor_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torque_sensor_mode(ref int handle, ref int sensor_mode);

		[DllImport("jakaAPI.dll", EntryPoint = "set_torque_sensor_comm", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torque_sensor_comm(ref int handle, int type, ref char[] ip_addr, int port);

		[DllImport("jakaAPI.dll", EntryPoint = "set_torque_sensor_comm", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torque_sensor_comm(ref int handle, int type, [MarshalAs(UnmanagedType.LPStr)] string ip_addr, int port);

		[DllImport("jakaAPI.dll", EntryPoint = "get_torque_sensor_comm", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torque_sensor_comm(ref int handle, ref int type, [MarshalAs(UnmanagedType.LPStr)] StringBuilder ip_addr, ref int port);

		[DllImport("jakaAPI.dll", EntryPoint = "set_torque_sensor_filter", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torque_sensor_filter(ref int handle, float torque_sensor_filter);

		[DllImport("jakaAPI.dll", EntryPoint = "get_torque_sensor_filter", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torque_sensor_filter(ref int handle, ref float torque_sensor_filter);

		[DllImport("jakaAPI.dll", EntryPoint = "set_torque_sensor_soft_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torque_sensor_soft_limit(ref int handle, JKTYPE.FTxyz torque_sensor_soft_limit);

		[DllImport("jakaAPI.dll", EntryPoint = "get_torque_sensor_soft_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torque_sensor_soft_limit(ref int handle, ref JKTYPE.FTxyz torque_sensor_soft_limit);

		[DllImport("jakaAPI.dll", EntryPoint = "get_torque_sensor_data", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torque_sensor_data(ref int handle, int type, ref JKTYPE.TorqSensorData data);

		[DllImport("jakaAPI.dll", EntryPoint = "zero_end_sensor", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int zero_end_sensor(ref int handle);

		[DllImport("jakaAPI.dll", EntryPoint = "set_compliant_speed_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_compliant_speed_limit(ref int handle, double vel, double angular_vel);

		[DllImport("jakaAPI.dll", EntryPoint = "get_compliant_speed_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_compliant_speed_limit(ref int handle, ref double vel, ref double angular_vel);

		[DllImport("jakaAPI.dll", EntryPoint = "set_torque_ref_point", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torque_ref_point(ref int handle, int refPoint);

		[DllImport("jakaAPI.dll", EntryPoint = "get_torque_ref_point", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torque_ref_point(ref int handle, ref int refPoint);

		[DllImport("jakaAPI.dll", EntryPoint = "set_end_sensor_sensitivity_threshold", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_end_sensor_sensitivity_threshold(ref int handle, JKTYPE.FTxyz value);

		[DllImport("jakaAPI.dll", EntryPoint = "get_end_sensor_sensitivity_threshold", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_end_sensor_sensitivity_threshold(ref int handle, ref JKTYPE.FTxyz value);

		[DllImport("jakaAPI.dll", EntryPoint = "set_force_stop_condition", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_force_stop_condition(ref int handle, ref JKTYPE.ForceStopConditionList condition);

		#endregion

		#region payload identify

		[DllImport("jakaAPI.dll", EntryPoint = "start_torq_sensor_payload_identify", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int start_torq_sensor_payload_identify(ref int i, ref JKTYPE.JointValue joint_pos);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_torq_sensor_identify_status", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torq_sensor_identify_status(ref int i, ref int identify_status);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_torq_sensor_payload_identify_result", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torq_sensor_payload_identify_result(ref int i, ref JKTYPE.PayLoad payload);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_torq_sensor_tool_payload", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_torq_sensor_tool_payload(ref int i, ref JKTYPE.PayLoad payload);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_torq_sensor_tool_payload", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_torq_sensor_tool_payload(ref int i, ref JKTYPE.PayLoad payload);
		
		#endregion

		#region tool drive
		[DllImport("jakaAPI.dll", EntryPoint = "get_admit_ctrl_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_admit_ctrl_config(ref int handle, ref JKTYPE.RobotAdmitCtrl admit_ctrl_cfg);

		[DllImport("jakaAPI.dll", EntryPoint = "disable_force_control", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int disable_force_control(ref int handle);

		[DllImport("jakaAPI.dll", EntryPoint = "set_vel_compliant_ctrl", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_vel_compliant_ctrl(ref int handle, ref JKTYPE.VelCom vel_cfg);

		[DllImport("jakaAPI.dll", EntryPoint = "set_admit_ctrl_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_admit_ctrl_config(ref int handle, int axis, int opt, double ftUser, double ftConstant, int ftNnormalTrack, double ftReboundFK);
		
		[DllImport("jakaAPI.dll", EntryPoint = "enable_admittance_ctrl", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int enable_admittance_ctrl(ref int handle, int enable_flag);

		[DllImport("jakaAPI.dll", EntryPoint = "enable_tool_drive", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int enable_tool_drive(ref int handle, int enable_flag);

		[DllImport("jakaAPI.dll", EntryPoint = "get_tool_drive_state", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tool_drive_state(ref int handle, ref int enable_flag, ref int state);

		[DllImport("jakaAPI.dll", EntryPoint = "get_tool_drive_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tool_drive_config(ref int handle, ref JKTYPE.RobotToolDriveCtrl cfg);

		[DllImport("jakaAPI.dll", EntryPoint = "set_tool_drive_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tool_drive_config(ref int handle, JKTYPE.ToolDriveConfig cfg);

		[DllImport("jakaAPI.dll", EntryPoint = "get_tool_drive_frame", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_tool_drive_frame(ref int handle, ref JKTYPE.FTFrameType frame);

		[DllImport("jakaAPI.dll", EntryPoint = "set_tool_drive_frame", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_tool_drive_frame(ref int handle, JKTYPE.FTFrameType frame);

		[DllImport("jakaAPI.dll", EntryPoint = "get_fusion_drive_sensitivity_level", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_fusion_drive_sensitivity_level(ref int handle, ref int level);

		[DllImport("jakaAPI.dll", EntryPoint = "set_fusion_drive_sensitivity_level", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_fusion_drive_sensitivity_level(ref int handle, int level);

		[DllImport("jakaAPI.dll", EntryPoint = "get_motion_limit_warning_range", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_motion_limit_warning_range(ref int handle, ref int warning_range);

		[DllImport("jakaAPI.dll", EntryPoint = "set_motion_limit_warning_range", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_motion_limit_warning_range(ref int handle, int warning_range);



		#endregion

		#region  force control

		[DllImport("jakaAPI.dll", EntryPoint = "set_ft_ctrl_frame", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_ft_ctrl_frame(ref int i, JKTYPE.FTFrameType ftFrame);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_ft_ctrl_frame", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_ft_ctrl_frame(ref int i,ref JKTYPE.FTFrameType ftFrame);

		[DllImport("jakaAPI.dll", EntryPoint = "set_compliant_type", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_compliant_type(ref int i, int sensor_compensation, int compliance_type);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_compliant_type", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_compliant_type(ref int i, ref int sensor_compensation, ref int compliance_type);
		
		[DllImport("jakaAPI.dll", EntryPoint = "set_compliance_condition", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_compliance_condition(ref int i, ref JKTYPE.FTxyz ft);

		[DllImport("jakaAPI.dll", EntryPoint = "set_approach_speed_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_approach_speed_limit(ref int i, double vel, double angularVel);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_approach_speed_limit", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_approach_speed_limit(ref int i, ref double vel, ref double angularVel);

		[DllImport("jakaAPI.dll", EntryPoint = "set_ft_tolerance", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_ft_tolerance(ref int i, double force, double torque);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_ft_tolerance", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_ft_tolerance(ref int i, ref double force, ref double torque);

		[DllImport("jakaAPI.dll", EntryPoint = "set_ft_ctrl_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_ft_ctrl_mode(ref int i, int mode);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_ft_ctrl_mode", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_ft_ctrl_mode(ref int i,ref int mode);

		[DllImport("jakaAPI.dll", EntryPoint = "set_ft_ctrl_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_ft_ctrl_config(ref int i, JKTYPE.AdmitCtrlType cfg);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_ft_ctrl_config", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_ft_ctrl_config(ref int i,ref JKTYPE.RobotAdmitCtrl cfg);


		#endregion

		#region  collision

		[DllImport("jakaAPI.dll", EntryPoint = "is_in_collision", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int is_in_collision(ref int handle, ref bool in_collision);

		[DllImport("jakaAPI.dll", EntryPoint = "collision_recover", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int collision_recover(ref int handle);
		[DllImport("jakaAPI.dll", EntryPoint = "clear_error", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int clear_error(ref int handle);
		[DllImport("jakaAPI.dll", EntryPoint = "set_collision_level", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int set_collision_level(ref int handle, int level);
		
		[DllImport("jakaAPI.dll", EntryPoint = "get_collision_level", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int get_collision_level(ref int handle, ref int level);
		
		#endregion
		
		#region  math

		[DllImport("jakaAPI.dll", EntryPoint = "kine_inverse", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int kine_inverse(ref int handle, ref JKTYPE.JointValue ref_pos, ref JKTYPE.CartesianPose cartesian_pose, ref JKTYPE.JointValue joint_pos);
		
		[DllImport("jakaAPI.dll", EntryPoint = "kine_forward", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int kine_forward(ref int handle, ref JKTYPE.JointValue joint_pos, ref JKTYPE.CartesianPose cartesian_pose);

		[DllImport("jakaAPI.dll", EntryPoint = "kine_inverse_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int kine_inverse_extend(ref int handle, ref JKTYPE.JointValue ref_pos, ref JKTYPE.CartesianPose cartesian_pose, ref JKTYPE.JointValue joint_pos, ref JKTYPE.CartesianPose tool, ref JKTYPE.CartesianPose userFrame);
		
		[DllImport("jakaAPI.dll", EntryPoint = "kine_forward_extend", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int kine_forward_extend(ref int handle, ref JKTYPE.JointValue joint_pos, ref JKTYPE.CartesianPose cartesian_pose, ref JKTYPE.CartesianPose tool, ref JKTYPE.CartesianPose userFrame);
		
		[DllImport("jakaAPI.dll", EntryPoint = "rpy_to_rot_matrix", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rpy_to_rot_matrix(ref int handle, ref JKTYPE.Rpy rpy, ref JKTYPE.RotMatrix rot_matrix);
		
		[DllImport("jakaAPI.dll", EntryPoint = "rot_matrix_to_rpy", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rot_matrix_to_rpy(ref int handle, ref JKTYPE.RotMatrix rot_matrix, ref JKTYPE.Rpy rpy);
		
		[DllImport("jakaAPI.dll", EntryPoint = "quaternion_to_rot_matrix", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int quaternion_to_rot_matrix(ref int handle, ref JKTYPE.Quaternion quaternion, ref JKTYPE.RotMatrix rot_matrix);
		
		[DllImport("jakaAPI.dll", EntryPoint = "rot_matrix_to_quaternion", ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
		public static extern int rot_matrix_to_quaternion(ref int handle, ref JKTYPE.RotMatrix rot_matrix, ref JKTYPE.Quaternion quaternion);
		
		#endregion


	}
}
