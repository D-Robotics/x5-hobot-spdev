#include "vp_sensors.h"

#define SENSOR_WIDTH  3840
#define SENSOR_HEIGHT  2160
#define SENSOE_FPS 30

static mipi_config_t sc850sl_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 4,
		.datatype = SENSOR_DATA_TYPE_RAW10,
		.fps = SENSOE_FPS,
		.mclk = 24,
		.mipiclk = 2880,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 4266,
		.framelenth = 2250,
		.settle = 0,
		.channel_num = 1,
		.channel_sel = {0},
	},
};

static camera_config_t sc850sl_camera_config = {
	.name = "sc850sl",
	.addr = 0x30,
	.sensor_mode = NORMAL_M,
	.fps = SENSOE_FPS,
	.format = SENSOR_DATA_TYPE_RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x07,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &sc850sl_mipi_config,
	.calib_lname = "disable",
};

static vin_node_attr_t sc850sl_vin_node_attr = {
	.cim_attr = {
		.mipi_rx = 0,
		.vc_index = 0,
		.ipi_channel = 1,
		.cim_isp_flyby = 1,
		.func = {
			.enable_frame_id = 1,
			.set_init_frame_id = 0,
			.hdr_mode = NOT_HDR,
			.time_stamp_en = 0,
		},
	},
	.lpwm_attr = {
		.enable = 0,
		.lpwm_chn_attr = {
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.trigger_source = 0,
				.trigger_mode = 0,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
		},
	},
};

static vin_attr_ex_t sc850sl_vin_attr_ex = {
	.vin_attr_ex_mask = 0x80,
	.mclk_ex_attr = {
		.mclk_freq = 27000000,
	},
};

static vin_ichn_attr_t sc850sl_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = SENSOR_DATA_TYPE_RAW10,
};

static vin_ochn_attr_t sc850sl_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = SENSOR_DATA_TYPE_RAW10,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t sc850sl_isp_attr = {
	.input_mode = 2, // 0: online, 2：offline
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t sc850sl_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t sc850sl_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

vp_sensor_config_t sc850sl_linear_3840x2160_raw10_30fps_4lane = {
	.chip_id_reg = 0x03107,
	.chip_id = 0x9d1e,
	.sensor_i2c_addr_list = {0x30},
	.sensor_name = "sc850sl-30fps",
	.config_file = "linear_3840x2160_raw10_30fps_4lane.c",
	.camera_config = &sc850sl_camera_config,
	.vin_ichn_attr = &sc850sl_vin_ichn_attr,
	.vin_node_attr = &sc850sl_vin_node_attr,
	.vin_attr_ex   = &sc850sl_vin_attr_ex,
	.vin_ochn_attr = &sc850sl_vin_ochn_attr,
	.isp_attr      = &sc850sl_isp_attr,
	.isp_ichn_attr = &sc850sl_isp_ichn_attr,
	.isp_ochn_attr = &sc850sl_isp_ochn_attr,
};
