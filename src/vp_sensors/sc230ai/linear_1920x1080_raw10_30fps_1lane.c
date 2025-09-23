#include "vp_sensors.h"

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT  1080
#define SENSOE_FPS 30
#define RAW10 0x2B

#define SENSOR_TYPE_NV12 8
#define ALIGN_UP(a, size) (((a) + (size)-1u) & (~((size)-1u)))

static mipi_config_t sc230ai_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 1,
		.datatype = RAW10,
		.fps = SENSOE_FPS,
		.mclk = 24,
		.mipiclk = 810,
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
		.linelenth = 2149,
		.framelenth = 1125 * 2,
		.settle = 22,
		.channel_num = 1,
		.channel_sel = {0},
	},
};

static camera_config_t sc230ai_camera_config = {
	.name = "sc230ai",
	.addr = 0x30,
	.sensor_mode = NORMAL_M,
	.fps = SENSOE_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.gpio_enable_bit = 0x07,
	.gpio_level_bit = 0x00,
	.mipi_cfg = &sc230ai_mipi_config,
	.calib_lname = "disable",
};

static vin_node_attr_t sc230ai_vin_node_attr = {
	.cim_attr = {
		.mipi_rx = 0,
		.vc_index = 0,
		.ipi_channel = 1,
		.cim_isp_flyby = 0,
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

static vin_attr_ex_t sc230ai_vin_attr_ex = {
	.vin_attr_ex_mask = 0x80,
	.mclk_ex_attr = {
		.mclk_freq = 24000000,
	},
};

static vin_ichn_attr_t sc230ai_vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t sc230ai_vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static isp_attr_t sc230ai_isp_attr = {
	.input_mode = 2, // 0: online, 2:offline
	.sensor_mode= ISP_NORMAL_M,
	.crop = {
		.x = 0,
		.y = 0,
		.h = SENSOR_HEIGHT,
		.w = SENSOR_WIDTH,
	},
};

static isp_ichn_attr_t sc230ai_isp_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.fmt = FRM_FMT_RAW,
	.bit_width = 10,
};

static isp_ochn_attr_t sc230ai_isp_ochn_attr = {
	.ddr_en = 1,
	.fmt = FRM_FMT_NV12,
	.bit_width = 8,
};

static n2d_config_t sc230ai_gpu2d_scale_crop_attr = {
	.command = N2D_SCALE_CROP,
	/* scale input */
	.input_width = {SENSOR_WIDTH},
	.input_height = {SENSOR_HEIGHT},
	.input_stride =  {ALIGN_UP(SENSOR_WIDTH, 16)},
	/* scale output */
	.output_width = SENSOR_WIDTH,
	.output_height = SENSOR_HEIGHT,
	.output_stride = ALIGN_UP(SENSOR_WIDTH, 16),

	.ninputs = 1, // number of inputs
	.output_format = 8, // // N2D_NV12
	.crop_x = 0,
	.crop_y = 0,
	.crop_width = 0,
	.crop_height = 0,
};


vp_sensor_config_t sc230ai_linear_1920x1080_raw10_30fps_1lane = {
	.chip_id_reg = 0x3107,
	.chip_id = 0xcb34,
	.sensor_i2c_addr_list = {0x30, 0x32},
	.sensor_name = "sc230ai-30fps",
	.config_file = "linear_1920x1080_raw10_30fps_1lane.c",
	.camera_config = &sc230ai_camera_config,
	.vin_ichn_attr = &sc230ai_vin_ichn_attr,
	.vin_node_attr = &sc230ai_vin_node_attr,
	.vin_attr_ex   = &sc230ai_vin_attr_ex,
	.vin_ochn_attr = &sc230ai_vin_ochn_attr,
	.isp_attr      = &sc230ai_isp_attr,
	.isp_ichn_attr = &sc230ai_isp_ichn_attr,
	.isp_ochn_attr = &sc230ai_isp_ochn_attr,
	.gpu2d_scale_crop_attr = &sc230ai_gpu2d_scale_crop_attr,
};
