// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vp_sensors.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>

#include <stdbool.h>

extern vp_sensor_config_t sc1330t_linear_1280x960_raw10_30fps_1lane;
extern vp_sensor_config_t irs2875_linear_208x1413_raw12_15fps_2lane;
extern vp_sensor_config_t sc230ai_linear_1920x1080_raw10_10fps_1lane;
extern vp_sensor_config_t sc230ai_linear_1920x1080_raw10_30fps_1lane;
extern vp_sensor_config_t sc132gs_linear_1088x1280_raw10_30fps_1lane;
extern vp_sensor_config_t sc035hgs_linear_640x480_raw10_30fps_1lane;
extern vp_sensor_config_t ov5640_linear_1920x1080_raw10_30fps_2lane;
extern vp_sensor_config_t f37_linear_1920x1080_raw10_30fps_1lane;
extern vp_sensor_config_t imx415_linear_3480x2160_raw10_30fps_4lane;
extern vp_sensor_config_t imx415_linear_3480x2160_raw10_30fps_2lane;
extern vp_sensor_config_t sc202cs_linear_1600x1200_raw10_30fps_1lane;
extern vp_sensor_config_t irs2381c_linear_224x1903_raw12_5fps_2lane;
extern vp_sensor_config_t imx219_linear_640x480_raw10_30fps_2lane;
extern vp_sensor_config_t imx219_linear_1632x1232_raw10_30fps_2lane;
extern vp_sensor_config_t imx219_linear_1920x1080_raw10_30fps_2lane;
extern vp_sensor_config_t imx219_linear_3264x2464_raw10_15fps_2lane;
extern vp_sensor_config_t imx219_linear_3264x2464_raw10_21fps_2lane;
extern vp_sensor_config_t ov5647_linear_640x480_raw10_60fps_2lane;
extern vp_sensor_config_t ov5647_linear_1280x960_raw10_30fps_2lane;
extern vp_sensor_config_t ov5647_linear_1920x1080_raw10_30fps_2lane;
extern vp_sensor_config_t ov5647_linear_2592x1944_raw10_15fps_2lane;
extern vp_sensor_config_t imx477_linear_1280x960_raw10_120fps_2lane;
extern vp_sensor_config_t imx477_linear_1920x1080_raw12_50fps_2lane;
extern vp_sensor_config_t imx477_linear_2016x1520_raw12_40fps_2lane;
extern vp_sensor_config_t imx477_linear_4000x3000_raw12_10fps_2lane;
extern vp_sensor_config_t sc035hgs_linear_640x480_raw10_30fps_2lane_vc0;
extern vp_sensor_config_t sc035hgs_linear_640x480_raw10_30fps_2lane_vc1;
extern vp_sensor_config_t sc231ai_linear_1920x1080_raw10_30fps_2lane;
extern vp_sensor_config_t imx586_linear_3480x2160_raw10_30fps_4lane;
extern vp_sensor_config_t os08c10_linear_3480x2160_raw12_30fps_2lane;

vp_sensor_config_t *vp_sensor_config_list[] = {
	&sc1330t_linear_1280x960_raw10_30fps_1lane,
	&irs2875_linear_208x1413_raw12_15fps_2lane,
	&sc230ai_linear_1920x1080_raw10_10fps_1lane,
	&sc230ai_linear_1920x1080_raw10_30fps_1lane,
	&sc132gs_linear_1088x1280_raw10_30fps_1lane,
	&sc035hgs_linear_640x480_raw10_30fps_1lane,
	&ov5640_linear_1920x1080_raw10_30fps_2lane,
	&f37_linear_1920x1080_raw10_30fps_1lane,
	&imx415_linear_3480x2160_raw10_30fps_2lane,
	&imx415_linear_3480x2160_raw10_30fps_4lane,
	&sc202cs_linear_1600x1200_raw10_30fps_1lane,
	&irs2381c_linear_224x1903_raw12_5fps_2lane,
	&imx219_linear_640x480_raw10_30fps_2lane,
	&imx219_linear_1632x1232_raw10_30fps_2lane,
	&imx219_linear_1920x1080_raw10_30fps_2lane,
	&imx219_linear_3264x2464_raw10_15fps_2lane,
	&imx219_linear_3264x2464_raw10_21fps_2lane,
	&ov5647_linear_640x480_raw10_60fps_2lane,
	&ov5647_linear_1280x960_raw10_30fps_2lane,
	&ov5647_linear_1920x1080_raw10_30fps_2lane,	
	&ov5647_linear_2592x1944_raw10_15fps_2lane,	
	&imx477_linear_1280x960_raw10_120fps_2lane,
	&imx477_linear_1920x1080_raw12_50fps_2lane,
	&imx477_linear_2016x1520_raw12_40fps_2lane,
	&imx477_linear_4000x3000_raw12_10fps_2lane,
	&sc035hgs_linear_640x480_raw10_30fps_2lane_vc0,
	&sc035hgs_linear_640x480_raw10_30fps_2lane_vc1,
	&sc231ai_linear_1920x1080_raw10_30fps_2lane,
	&imx586_linear_3480x2160_raw10_30fps_4lane,
	&os08c10_linear_3480x2160_raw12_30fps_2lane,
};

uint32_t vp_get_sensors_list_number() {
	return sizeof(vp_sensor_config_list) / sizeof(vp_sensor_config_list[0]);
}

void vp_show_sensors_list() {
	int num = 0;

	num = vp_get_sensors_list_number();
	for (int i = 0; i < num; i++) {
		printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n",
		i, vp_sensor_config_list[i]->sensor_name,
		vp_sensor_config_list[i]->config_file);
	}
}

void vp_show_sensors_list_vse_limit(uint32_t width_limit, uint32_t height_limit)
{
	int num = 0;
	int quarter_of_vse_max_resolution = width_limit * height_limit / 4;

	num = vp_get_sensors_list_number();
	for (int i = 0; i < num; i++) {
		vp_sensor_config_t* sensor_config = vp_sensor_config_list[i];
		int width_tmp = sensor_config->camera_config->width;
		int height_tmp = sensor_config->camera_config->height;
		if (width_tmp * height_tmp < quarter_of_vse_max_resolution) {
			continue;
		}
		printf("index: %d  sensor_name: %-16s \tconfig_file:%s\n",
			i, vp_sensor_config_list[i]->sensor_name,
			vp_sensor_config_list[i]->config_file);
	}
}

// Check system endianness
static int is_little_endian() {
	uint16_t test = 0x0001;
	return *((uint8_t*)(&test)) == 0x01;
}

// Endianness conversion for 32-bit integer
static int32_t convert_endianness_int32(int32_t value) {
	if (is_little_endian()) {
		// Convert from little endian to big endian
		return ((value >> 24) & 0x000000FF)
			| ((value >> 8) & 0x0000FF00)
			| ((value << 8) & 0x00FF0000)
			| ((value << 24) & 0xFF000000);
	} else {
		// Convert from big endian to little endian
		return value;
	}
}

#define GPIO_EXPORT_PATH "/sys/class/gpio/export"
#define GPIO_UNEXPORT_PATH "/sys/class/gpio/unexport"

// Function to export a GPIO
static int gpio_export(int gpio_number) {
	FILE *fp;
	fp = fopen(GPIO_EXPORT_PATH, "w");
	if (fp == NULL) {
		printf("Error opening GPIO export file for writing\n");
		return -1;
	}
	fprintf(fp, "%d", gpio_number);
	fclose(fp);
	return 0;
}

// Function to unexport a GPIO
static int gpio_unexport(int gpio_number) {
	FILE *fp;
	fp = fopen(GPIO_UNEXPORT_PATH, "w");
	if (fp == NULL) {
		printf("Error opening GPIO unexport file for writing\n");
		return -1;
	}
	fprintf(fp, "%d", gpio_number);
	fclose(fp);
	return 0;
}

// Function to set GPIO direction
static int gpio_set_direction(int gpio_number, const char *direction) {
	char filename[256];
	FILE *fp;
	int elapsed_time = 0;

	snprintf(filename, sizeof(filename), "/sys/class/gpio/gpio%d/direction", gpio_number);

	while ((fp = fopen(filename, "w")) == NULL) {
        if (elapsed_time >= 100) {
            fprintf(stderr, "Timeout: Failed to open %s after 100ms: %s\n", filename, strerror(errno));
            return -1;
        }
        usleep(1000);
        elapsed_time += 1;
    }

	fprintf(fp, "%s", direction);
	fclose(fp);
	return 0;
}

// Function to set GPIO value
static int gpio_set_value(int gpio_number, int value) {
	char filename[256];
	FILE *fp;
	snprintf(filename, sizeof(filename), "/sys/class/gpio/gpio%d/value", gpio_number);
	fp = fopen(filename, "w");
	if (fp == NULL) {
		printf("Error opening GPIO value file for writing\n");
		return -1;
	}
	fprintf(fp, "%d", value);
	fclose(fp);
	return 0;
}

#define GPIO_PATH_TEMPLATE "/sys/class/gpio/gpio%d"
#define POLL_INTERVAL_US 10000
#define MAX_WAIT_TIME_MS 100

static bool wait_for_gpio_export(int gpio_number) {
    char gpio_path[64];
    snprintf(gpio_path, sizeof(gpio_path), GPIO_PATH_TEMPLATE, gpio_number);

    for (int i = 0; i < (MAX_WAIT_TIME_MS * 1000) / POLL_INTERVAL_US; i++) {
		if (access(gpio_path, F_OK) == 0)
            return true; 
        usleep(POLL_INTERVAL_US);
    }

    return false;
}

static int enable_sensor_pin(int gpio_number, int active)
{
	static int last_gpio_number = -1;

    if (gpio_number == last_gpio_number) {
        return 0;
    }
	
	// Export the GPIO
	if (gpio_export(gpio_number) != 0) {
		printf("Failed to export GPIO\n");
		return -1;
	}

	if (!wait_for_gpio_export(gpio_number)) {
        printf("GPIO directory not created in time for GPIO %d\n", gpio_number);
        gpio_unexport(gpio_number); 
        return -1;
    }

	// Set GPIO direction to output
	if (gpio_set_direction(gpio_number, "out") != 0) {
		printf("Failed to set GPIO direction\n");
		return -1;
	}

	usleep(30 * 1000);

	/* gpio level should be keep same with sensor driver power_on api */

	// Set GPIO value to active
	if (gpio_set_value(gpio_number, active) != 0) {
		printf("Failed to set GPIO value\n");
		return -1;
	}

	usleep(30 * 1000);

	// Set GPIO value to 1 - active
	if (gpio_set_value(gpio_number,  (1 - active)) != 0) {
		printf("Failed to set GPIO value\n");
		return -1;
	}

	usleep(30 * 1000);

	// Set GPIO value to active
	if (gpio_set_value(gpio_number, active) != 0) {
		printf("Failed to set GPIO value\n");
		return -1;
	}

	usleep(30 * 1000);

	// Unexport the GPIO
	if (gpio_unexport(gpio_number) != 0) {
		printf("Failed to unexport GPIO\n");
		return -1;
	}

	last_gpio_number = gpio_number;

	return 0;
}
//9
static void read_mipi_info_from_device_tree(const int device, struct mipi_properties *properties) {
	#define MIPI_DEVICE_COUNT 4
	const char *mipi_device_tree_node_suffixs [MIPI_DEVICE_COUNT]= {"3d060000", "3d070000","3d080000", "3d090000"};
	if(device > MIPI_DEVICE_COUNT){
		printf("Error device %d exceed max valud %d\n", device, MIPI_DEVICE_COUNT);
		return;
	}
	const char *node_suffix = mipi_device_tree_node_suffixs[device];
	memset(properties, 0, sizeof(struct mipi_properties));

	snprintf(properties->device_path, sizeof(properties->device_path),
		"/proc/device-tree/soc/cam/mipi_host@%s", node_suffix);

	DIR *dir = opendir(properties->device_path);
	if (dir == NULL) {
		printf("Error opening directory: %s\n", properties->device_path);
		return;
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) { // Regular file
			char filename[VP_MAX_BUF_SIZE] = {0};
			int ret = snprintf(filename, sizeof(filename), "%s/%s", properties->device_path, entry->d_name);
			if (ret < 0 || ret >= sizeof(filename)) {
				printf("Error: Failed to set filename\n");
				return;
			}
			FILE *fp = fopen(filename, "rb");
			if (fp != NULL) {
				if (strcmp(entry->d_name, "status") == 0) {
					fread(&properties->status, sizeof(char), VP_MAX_BUF_SIZE, fp);
				}  else if (strcmp(entry->d_name, "pinctrl-names") == 0) {
					fread(&properties->pinctrl_names, sizeof(char), VP_MAX_BUF_SIZE, fp);
				}  else if (strcmp(entry->d_name, "pinctrl-0") == 0) {
					fread(&properties->pinctrl_0, sizeof(int32_t), sizeof(properties->pinctrl_0) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->pinctrl_0) / sizeof(int32_t); ++i)
						properties->pinctrl_0[i] = convert_endianness_int32(properties->pinctrl_0[i]);
				} else if (strcmp(entry->d_name, "pinctrl-1") == 0) {
					fread(&properties->pinctrl_1, sizeof(int32_t), sizeof(properties->pinctrl_1) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->pinctrl_1) / sizeof(int32_t); ++i)
						properties->pinctrl_1[i] = convert_endianness_int32(properties->pinctrl_1[i]);
				} else if (strcmp(entry->d_name, "snrclk-idx") == 0) {
					fread(&properties->snrclk_idx, sizeof(int32_t), sizeof(properties->snrclk_idx) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->snrclk_idx) / sizeof(int32_t); ++i)
						properties->snrclk_idx[i] = convert_endianness_int32(properties->snrclk_idx[i]);
				}

				// Close the file
				fclose(fp);
			}
		}
	}

	closedir(dir);
}
static void read_vcon_info_from_device_tree(const int device, struct vcon_properties *properties) {
	memset(properties, 0, sizeof(struct vcon_properties));

	snprintf(properties->device_path, sizeof(properties->device_path),
		"/proc/device-tree/soc/cam/vcon@%d", device);

	DIR *dir = opendir(properties->device_path);
	if (dir == NULL) {
		printf("Error opening directory: %s\n", properties->device_path);
		return;
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) { // Regular file
			char filename[VP_MAX_BUF_SIZE] = {0};
			int ret = snprintf(filename, sizeof(filename), "%s/%s", properties->device_path, entry->d_name);
			if (ret < 0 || ret >= sizeof(filename)) {
				printf("Error: Failed to set filename\n");
				return;
			}

			FILE *fp = fopen(filename, "rb");
			if (fp != NULL) {
				if (strcmp(entry->d_name, "compatible") == 0) {
					fread(&properties->compatible, sizeof(char), VP_MAX_BUF_SIZE, fp);
				} else if (strcmp(entry->d_name, "status") == 0) {
					fread(&properties->status, sizeof(char), VP_MAX_BUF_SIZE, fp);
				}  else if (strcmp(entry->d_name, "pinctrl-names") == 0) {
					fread(&properties->pinctrl_names, sizeof(char), VP_MAX_BUF_SIZE, fp);
				} else if (strcmp(entry->d_name, "type") == 0) {
					fread(&properties->type, sizeof(int32_t), 1, fp);
					properties->type = convert_endianness_int32(properties->type);
				} else if (strcmp(entry->d_name, "bus") == 0) {
					fread(&properties->bus, sizeof(int32_t), 1, fp);
					properties->bus = convert_endianness_int32(properties->bus);
				} else if (strcmp(entry->d_name, "rx_phy") == 0) {
					fread(&properties->rx_phy, sizeof(int32_t), sizeof(properties->rx_phy) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->rx_phy) / sizeof(int32_t); ++i)
						properties->rx_phy[i] = convert_endianness_int32(properties->rx_phy[i]);
				} else if (strcmp(entry->d_name, "pinctrl-0") == 0) {
					fread(&properties->pinctrl_0, sizeof(int32_t), sizeof(properties->pinctrl_0) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->pinctrl_0) / sizeof(int32_t); ++i)
						properties->pinctrl_0[i] = convert_endianness_int32(properties->pinctrl_0[i]);
				} else if (strcmp(entry->d_name, "gpio_oth") == 0) {
					fread(&properties->gpio_oth, sizeof(int32_t), sizeof(properties->gpio_oth) / sizeof(int32_t), fp);
					for (int i = 0; i < sizeof(properties->gpio_oth) / sizeof(int32_t); ++i)
						properties->gpio_oth[i] = convert_endianness_int32(properties->gpio_oth[i]);
				}

				// Close the file
				fclose(fp);
			}
		}
	}

	closedir(dir);
}

static int32_t vp_i2c_read_reg16_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint16_t *value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[32] = {0};
	uint8_t readbuf[32] = {0};
	struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};
	char filename[20];
	int file;

	// Open the I2C bus
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
	file = open(filename, O_RDWR);
	if (file < 0) {
		perror("Failed to open the I2C bus");
		return -1;
	}

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 2;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = I2C_M_RD;
	data.msgs[1].buf = readbuf;

	ret = ioctl(file, I2C_RDWR, (uint64_t)&data);
	if (ret < 0) {
		// perror("Failed to read from the I2C bus");
		*value = 0;
		close(file);
		return -1;
	}

	*value = (uint16_t)((readbuf[0] << 8) | readbuf[1]);

	// Close the I2C bus
	close(file);

	return 0;
}


static int32_t vp_i2c_read_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t *value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[32] = {0};
	uint8_t readbuf[32] = {0};
	struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};
	char filename[20];
	int file;

	// Open the I2C bus
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
	file = open(filename, O_RDWR);
	if (file < 0) {
		perror("Failed to open the I2C bus");
		return -1;
	}

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 2;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = readbuf;

	ret = ioctl(file, I2C_RDWR, (uint64_t)&data);
	if (ret < 0) {
		// perror("Failed to read from the I2C bus");
		*value = 0;
		close(file);
		return -1;
	}
	*value = readbuf[0];

	// Close the I2C bus
	close(file);

	return 0;
}

static int32_t read_chip_id(vcon_propertie_t vcon_props, vp_sensor_config_t *sensor_config, uint32_t addr, int32_t *chip_id) {

	if(sensor_config->chip_id >> 8 == 0) {
		// 读取 8 位 chip ID
		if (vp_i2c_read_reg16_data8(vcon_props.bus, addr, sensor_config->chip_id_reg, (uint8_t*)chip_id) == 0) {
			if ((sensor_config->chip_id == 0x005A) // 如果有的 sensor 本身读不到ID，但是又想要使用它，就把 sensor 的 chip_id 设为 0x005A
			|| ((chip_id[0] & 0xFF) == (sensor_config->chip_id & 0xFF))) {
				return 0;
			}
		}
	} else {
		// 读取 16 位 chip ID
		if (vp_i2c_read_reg16_data16(vcon_props.bus, addr, sensor_config->chip_id_reg, (uint16_t*)chip_id) == 0) {
			if ((sensor_config->chip_id == 0xA55A) // 如果有的 sensor 本身读不到ID，但是又想要使用它，就把 sensor 的 chip_id 设为 0xA55A
			|| ((chip_id[0] & 0xFFFF) == (sensor_config->chip_id & 0xFFFF))) {
				return 0;
			}
		}
	}
	return -1;
}

// Function to check sensor register value
static int32_t check_sensor_reg_value(vcon_propertie_t vcon_props,
		vp_sensor_config_t *sensor_config) {
	int i = 0;
	// Read sensor chip ID register
	int32_t chip_id = 0;
	uint32_t addr = sensor_config->camera_config->addr;

	// Add camera_config->addr to sensor_i2c_addr_list
	for (i = 1; i < 8; i++) {
		if (sensor_config->sensor_i2c_addr_list[i] == 0) {
			sensor_config->sensor_i2c_addr_list[i] = addr;
			break;
		}
	}

	// Try reading chip ID using sensor_i2c_addr_list
	for (i = 0; i < 8; i++) {
		addr = sensor_config->sensor_i2c_addr_list[i];
		if (addr == 0)
			continue;
		if (read_chip_id(vcon_props, sensor_config, addr, &chip_id) == 0) {
			// Update sensor address to the one successfully read from
			sensor_config->camera_config->addr = addr;
			// Update mipi rx phy
			// 修正配置中的 sensor 使用的 mipi rx phy
			// 此处的修改并不一定是最终修改，在vpp 的impl 的 param 设置中会根据具体情况再次修改
			sensor_config->vin_node_attr->cim_attr.mipi_rx = vcon_props.rx_phy[1];
			return 0;
		} else {
			printf("WARN: Sensor Name: %s, Expected Chip ID: 0x%02X, Actual Chip ID Read: 0x%02X\n",
					sensor_config->sensor_name, sensor_config->chip_id & 0x0000FFFF, chip_id);
			return -1;
		}

	}

	// If none of the addresses worked
	// printf("Failed to read sensor chip ID register\n");
	return -1;
}

// Function to write frequency to MIPI host
static void write_mipi_host_freq(int mipi_host, int freq)
{
	char path[256];
	FILE *file;

	// Construct path to the file
	snprintf(path, 256, "/sys/class/vps/mipi_host%d/param/snrclk_freq", mipi_host);

	// Open the file for writing
	file = fopen(path, "w");
	if (file) {
		// Write frequency to the file
		fprintf(file, "%d", freq);
		fclose(file);
	}
}

// Function to enable MIPI host clock
static void enable_mipi_host_clock(int mipi_host, int enable)
{
	char path[256];
	FILE *file;

	// Construct path to the file
	snprintf(path, 256, "/sys/class/vps/mipi_host%d/param/snrclk_en", mipi_host);

	// Open the file for writing
	file = fopen(path, "w");
	if (file) {
		// Write enable value to the file
		fprintf(file, "%d", enable);
		fclose(file);
	}
}

static int check_mipi_host_status(int mipi_host) {
	char file_path[100];
	snprintf(file_path, sizeof(file_path), "/sys/class/vps/mipi_host%d/status/cfg", mipi_host);

	FILE *file = fopen(file_path, "r");
	if (file == NULL) {
		printf("Failed to open %s: %s\n", file_path, strerror(errno));
		return 0;
	}

	char first_line[256];
	if (fgets(first_line, sizeof(first_line), file) == NULL) {
		perror("Failed to read file");
		fclose(file);
		return 0;
	}

	fclose(file);

	first_line[strcspn(first_line, "\n")] = '\0';

	// 判断第一行内容是否为 "not inited"
	if (strcmp(first_line, "not inited") == 0) {
		return 1;
	} else {
		return 0;
	}
}

int get_board_id(char *data, size_t size)
{
	const char *board_id_file = "/sys/class/socinfo/board_id";
	FILE *fp = fopen(board_id_file, "r");
	if (fp == NULL) {
		printf("[ERROR] open file %s failed.\n", board_id_file);
		return -1;
	}

	if (fgets(data, size, fp) == NULL) {
		printf("[ERROR] read file %s failed.\n", board_id_file);
		fclose(fp);
		return -1;
	}
	fclose(fp);

	// Remove trailing newline
	size_t len = strlen(data);
	if (len > 0 && data[len - 1] == '\n') {
		data[len - 1] = '\0';
	}

	// Trim leading and trailing whitespace
	char *start = data;
	while (isspace((unsigned char)*start)) {
		start++;
	}

	char *end = data + strlen(data) - 1;
	while (end > start && isspace((unsigned char)*end)) {
		end--;
	}

	// Null-terminate the trimmed string
	*(end + 1) = '\0';

	// Move the trimmed string to the start of the buffer
	if (start != data) {
		memmove(data, start, end - start + 2);
	}

	return 0;
}

static int32_t vp_sensor_mipi_host_mclk_is_not_configed(int csi_index){
	int mclk_is_not_configed = 0;
	struct mipi_properties mipi_property;

	read_mipi_info_from_device_tree(csi_index, &mipi_property);
	if(strlen(mipi_property.pinctrl_names) == 0){
			mclk_is_not_configed = 1;
			printf("mipi mclk is not configed.\n");
		}else{

			printf("mipi mclk is configed.\n");
		}
	return mclk_is_not_configed;
}

vp_sensor_config_t *vp_get_sensor_config_by_mipi_host(int32_t mipi_host_index,
	vp_csi_config_t* csi_config,int sensor_height,int sensor_width,int sensor_fps)
{
	int32_t ret = 0, j = 0;
	uint32_t frequency = 24000000;
	int32_t sensor_n = 0;
	int32_t sensor_list[10] = {-1};

	struct vcon_properties vcon_props_array[VP_MAX_VCON_NUM];

	if (check_mipi_host_status(mipi_host_index) == 0) {
		printf("Mipi csi%d has been used, please use other Cam interfaces\n", mipi_host_index);
		return NULL;
	}
	int mclk_is_not_configed = vp_sensor_mipi_host_mclk_is_not_configed(mipi_host_index);
	read_vcon_info_from_device_tree(mipi_host_index, &vcon_props_array[mipi_host_index]);

	if(!mclk_is_not_configed) {
		/* enable mclk */
		write_mipi_host_freq(mipi_host_index, frequency);
		enable_mipi_host_clock(mipi_host_index, 1);
	}

	csi_config->mclk_is_not_configed = mclk_is_not_configed;

	printf("Searching camera sensor on device: %s ", vcon_props_array[mipi_host_index].device_path);
	printf("i2c bus: %d ", vcon_props_array[mipi_host_index].bus);
	printf("mipi rx phy: %d\n", vcon_props_array[mipi_host_index].rx_phy[1]);

	// 如果该vcon使能了，检测该vcon上是否有连接 sensor
	if (vcon_props_array[mipi_host_index].status[0] == 'o') { // okay
		// 检测该vcon上连接的 sensor
		for (j = 0; j < vp_get_sensors_list_number(); j++) {
			/*enable gpio_oth, enable camera sensor gpio, maybe pwd/reset gpio */
			for (int k = 0; k < 8; ++k) {
				if (vcon_props_array[mipi_host_index].gpio_oth[k] != 0) {
					if ((vp_sensor_config_list[j]->camera_config->gpio_enable_bit & (1 << k)) != 0) {
						enable_sensor_pin(vcon_props_array[mipi_host_index].gpio_oth[k],
							(1 - vp_sensor_config_list[j]->camera_config->gpio_level_bit));
					}
				}
			}

			int ret = check_sensor_reg_value(vcon_props_array[mipi_host_index], vp_sensor_config_list[j]);
			if (ret == 0) {
				if ((sensor_height == -1 || vp_sensor_config_list[j]->camera_config->height == sensor_height) && 
    			  (sensor_width == -1 || vp_sensor_config_list[j]->camera_config->width == sensor_width) && 
    			  (sensor_fps == -1 || vp_sensor_config_list[j]->camera_config->fps == sensor_fps)) {
					printf("[%d] INFO: Found sensor name:%s on mipi rx csi %d, "
						"i2c addr 0x%x, config_file:%s\n",
					sensor_n,
					vp_sensor_config_list[j]->sensor_name,
					vcon_props_array[mipi_host_index].rx_phy[1],
					vp_sensor_config_list[j]->camera_config->addr,
					vp_sensor_config_list[j]->config_file);
					if(sensor_n < 10)
					{
						sensor_list[sensor_n] = j;
						sensor_n++;
					}
					else
						break;
				}
			}
		}

		if(sensor_n > 0)
		{
			j = 0;
			if(sensor_n > 1)
			{
				/* 这里需要更改一下，不能手动选， 看用 fps 匹配的策略 还是 什么方法 ，现在明确，优先完整匹配，但如果不匹配，就选择最接近的一项，然后输出log，进行提醒。*/
				int best_index = 0;
				int max_resolution = 0;
				int max_fps = 0;
				for (int k = 0; k < sensor_n; k++) 
				{
					vp_sensor_config_t *config = vp_sensor_config_list[sensor_list[k]];
					int current_res = config->camera_config->width * config->camera_config->height;
					int current_fps = config->camera_config->fps;

					// 优先比较分辨率，若相同则比较 FPS
					if (current_res > max_resolution || 
					(current_res == max_resolution && current_fps > max_fps)) 
					{
						best_index = k;
						max_resolution = current_res;
						max_fps = current_fps;
					}
				}
				j = best_index;
				printf("Auto-selected sensor: %s (Resolution: %dx%d@%dfps)\n",
					vp_sensor_config_list[sensor_list[j]]->sensor_name,
					vp_sensor_config_list[sensor_list[j]]->camera_config->width,
					vp_sensor_config_list[sensor_list[j]]->camera_config->height,
					vp_sensor_config_list[sensor_list[j]]->camera_config->fps);
			}
			/* Fixed Mipi host */
			vp_sensor_config_list[sensor_list[j]]->vin_node_attr->cim_attr.mipi_rx = vcon_props_array[mipi_host_index].rx_phy[1];
			return vp_sensor_config_list[sensor_list[j]];
		}
	}
	return NULL;
}
