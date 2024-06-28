#include "bm_common.h"

#include "bm_boot_info.tmh"
int bmdrv_get_ddr_ecc_enable(struct bm_device_info *bmdi, unsigned int chip_id)
{
	int ecc_enable = 0;

	switch (chip_id) {
	case 0x1682:
		break;
	case 0x1684:
		ecc_enable = bmdi->boot_info.ddr_ecc_enable;
		break;
	case 0x1686:
		ecc_enable = bmdi->boot_info.ddr_ecc_enable;
		break;
	default:
		return 0;
	}

	return ecc_enable;
}

int bmdrv_misc_info_init(struct bm_device_info *bmdi)
{
	struct bm_misc_info *misc_info = &bmdi->misc_info;
	struct chip_info *cinfo = &bmdi->cinfo;
	int domain_nr = 0;
	int domain_bdf = 0;
	unsigned char bus, dev, fn;

	domain_nr = 0;
	bus = (u8)(bmdi->bus_number);
	dev = (u8)(bmdi->device_number);
	fn = (u8)(bmdi->function_number);

	domain_bdf = (domain_nr << 16) | ((bus&0xff) << 8) | ((dev&0x1f) << 5) | (fn&0x7);

	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "[bmdrv_misc_info_init] chip_id = 0x%x\n", cinfo->chip_id);

	switch (cinfo->chip_id) {
	case 0x1682:
		misc_info->chipid_bit_mask = BM1682_CHIPID_BIT_MASK;
		break;
	case 0x1684:
		misc_info->chipid_bit_mask = BM1684_CHIPID_BIT_MASK;
		break;
	case 0x1686:
		misc_info->chipid_bit_mask = BM1684X_CHIPID_BIT_MASK;
		break;
	default:
		return -1;
	}
	misc_info->chipid = cinfo->chip_id;
	misc_info->pcie_soc_mode = BM_DRV_PCIE_MODE;
	misc_info->ddr_ecc_enable = bmdrv_get_ddr_ecc_enable(bmdi, cinfo->chip_id);
	misc_info->domain_bdf = domain_bdf;
	misc_info->driver_version = BM_DRIVER_VERSION;
	misc_info->ddr0a_size = bmdi->boot_info.ddr_0a_size;
	misc_info->ddr0b_size = bmdi->boot_info.ddr_0b_size;
	misc_info->ddr1_size = bmdi->boot_info.ddr_1_size;
	misc_info->ddr2_size = bmdi->boot_info.ddr_2_size;
	misc_info->board_version = bmdi->cinfo.board_version;
	if (((bmdi->boot_info.boot_info_version >> 16) & 0xffff) == BOOT_INFO_VERSION) {
		cinfo->a53_enable = bmdi->boot_info.append.append_v1.a53_enable;
		cinfo->heap2_size = bmdi->boot_info.append.append_v1.heap2_size;
		if (cinfo->heap2_size > 0x100000000)
			cinfo->heap2_size = 0x0;
		if (cinfo->a53_enable != 0)
			cinfo->a53_enable = 1;
	} else {
		cinfo->a53_enable = 1;
		cinfo->heap2_size = 0;
	}
	misc_info->a53_enable = cinfo->a53_enable;
    TraceEvents(TRACE_LEVEL_INFORMATION,
                TRACE_DEVICE,
                "a53 = %d, heap2 = 0x%llx\n",
                cinfo->a53_enable,
                cinfo->heap2_size);
	return 0;
}

void bmdrv_dump_bootinfo(struct bm_device_info *bmdi)
{
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"boot_info are:\n");
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"deadbeef = 0x%x\n", bmdi->boot_info.deadbeef);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_ecc_enable = %d\n", bmdi->boot_info.ddr_ecc_enable);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_0a_size = 0x%llx\n", bmdi->boot_info.ddr_0a_size);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_0b_size = 0x%llx\n", bmdi->boot_info.ddr_0b_size);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_1_size = 0x%llx\n", bmdi->boot_info.ddr_1_size);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_2_size = 0x%llx\n", bmdi->boot_info.ddr_2_size);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_mode = %d\n", bmdi->boot_info.ddr_mode);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_vendor_id = %d\n", bmdi->boot_info.ddr_vendor_id);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"ddr_rank_mode = %d\n", bmdi->boot_info.ddr_rank_mode);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"fan_exist = %d\n", bmdi->boot_info.fan_exist);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"tpu_min_clk = %d Mhz\n", bmdi->boot_info.tpu_min_clk);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"tpu_max_clk = %d Mhz\n", bmdi->boot_info.tpu_max_clk);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"max_board_power = %d Wa\n", bmdi->boot_info.max_board_power);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"temp_sensor_exist = %d\n", bmdi->boot_info.temp_sensor_exist);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"tpu_power_sensor_exist = %d\n", bmdi->boot_info.tpu_power_sensor_exist);
	TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DEVICE,"board_power_sensor_exist = %d\n", bmdi->boot_info.board_power_sensor_exist);
    TraceEvents(TRACE_LEVEL_INFORMATION,
                TRACE_DEVICE,
                "boot_info_version = 0x%x\n",
                bmdi->boot_info.boot_info_version);
	if (((bmdi->boot_info.boot_info_version >> 16) & 0xffff) == BOOT_INFO_VERSION) {
        TraceEvents(TRACE_LEVEL_INFORMATION,
                    TRACE_DEVICE,
                    "boot_info_version = 0x%x\n",
                    bmdi->boot_info.boot_info_version & 0xffff);
	}
	if (bmdi->boot_info.boot_info_version == BOOT_INFO_VERSION_V1) {
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,"a53 enable = 0x%x\n", bmdi->boot_info.append.append_v1.a53_enable);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,"heap2 size = 0x%llx\n", bmdi->boot_info.append.append_v1.heap2_size);
	}

}

int bmdrv_set_default_boot_info(struct bm_device_info *bmdi)
{
	int board_version = 0;
	u8 board_type, hw_version = 0;
	int function_num = 0;

	board_version = bmdi->cinfo.board_version;
	board_type = (u8)((board_version >> 8) & 0xff);
	hw_version = (u8)(board_version & 0xff);
	if (bmdi->cinfo.chip_index > 0)
		function_num = bmdi->cinfo.chip_index;
	else
        function_num = bmdi->function_number;

	if (bmdi->boot_info.deadbeef != 0xdeadbeef) {
		switch (board_type) {
		case BOARD_TYPE_EVB:
		case BOARD_TYPE_SC5:
			if (hw_version == 0x0) {
				bmdi->boot_info.ddr_rank_mode = 0x1;
				bmdi->boot_info.ddr_1_size = 0x80000000;
				bmdi->boot_info.ddr_2_size = 0x80000000;
			} else {
				bmdi->boot_info.ddr_rank_mode = 0x3;
				bmdi->boot_info.ddr_1_size = 0x100000000;
				bmdi->boot_info.ddr_2_size = 0x100000000;
			}
			bmdi->boot_info.board_power_sensor_exist = 1;
			bmdi->boot_info.fan_exist = 1;
			bmdi->boot_info.max_board_power = 30;
			break;
		case BOARD_TYPE_SC5_PLUS:
			bmdi->boot_info.ddr_rank_mode = 0x3;
			bmdi->boot_info.ddr_1_size = 0x100000000;
			bmdi->boot_info.ddr_2_size = 0x100000000;
			bmdi->boot_info.fan_exist = 0;
			bmdi->boot_info.max_board_power = 75;
			if (function_num == 0x0)
				bmdi->boot_info.board_power_sensor_exist = 1;
			else
				bmdi->boot_info.board_power_sensor_exist = 0;
			break;
		case BOARD_TYPE_SC5_H:
			bmdi->boot_info.ddr_rank_mode = 0x3;
			bmdi->boot_info.ddr_1_size = 0x100000000;
			bmdi->boot_info.ddr_2_size = 0x100000000;
			bmdi->boot_info.fan_exist = 1;
			bmdi->boot_info.max_board_power = 30;
			bmdi->boot_info.board_power_sensor_exist = 1;
			break;
		case BOARD_TYPE_SM5_P:
		case BOARD_TYPE_SM5_S:
			if (hw_version >= 0x11) {
				bmdi->boot_info.ddr_rank_mode = 0x3;
				bmdi->boot_info.ddr_1_size = 0x100000000;
				bmdi->boot_info.ddr_2_size = 0x100000000;
			} else {
				bmdi->boot_info.ddr_rank_mode = 0x1;
				bmdi->boot_info.ddr_1_size = 0x80000000;
				bmdi->boot_info.ddr_2_size = 0x80000000;
			}
			bmdi->boot_info.board_power_sensor_exist = 0;
			bmdi->boot_info.fan_exist = 1;
			bmdi->boot_info.max_board_power = 25;
			break;
		case BOARD_TYPE_SE5:
		case BOARD_TYPE_SA6:
		default:
            TraceEvents(TRACE_LEVEL_INFORMATION,
                        TRACE_DEVICE,
                        "unknow board type = %d\n",
                        board_type);
			return -1;
		}
	bmdi->boot_info.ddr_ecc_enable = 0;
	bmdi->boot_info.ddr_0a_size = 0x80000000;
	bmdi->boot_info.ddr_0b_size = 0x80000000;
	bmdi->boot_info.ddr_mode = 0x1;
	bmdi->boot_info.ddr_vendor_id = 0;
	bmdi->boot_info.tpu_min_clk = 75;
	bmdi->boot_info.tpu_max_clk = 550;
	bmdi->boot_info.temp_sensor_exist = 1;
	bmdi->boot_info.tpu_power_sensor_exist = 1;
	bmdi->boot_info.deadbeef = 0xdeadbeef;
	bmdi->boot_info.boot_info_version = BOOT_INFO_VERSION_V1;
	bmdi->boot_info.append.append_v1.a53_enable = 0x1;
	bmdi->boot_info.append.append_v1.heap2_size = 0x40000000;
	}
    TraceEvents(TRACE_LEVEL_INFORMATION,
                TRACE_DEVICE,
                "board verion = 0x%x, bord_type = 0x%x, hw_version = 0x%x\n",
		board_version, board_type, hw_version);
	return 0;
}

int bmdrv_check_bootinfo(struct bm_device_info *bmdi)
{
	int board_version = 0;
	u8 board_type = 0;
	int need_update = 0;
	int ret = 0;
	int function_num = 0;

	board_version = bmdi->cinfo.board_version;
	board_type = (u8)((board_version >> 8) & 0xff);
	if (bmdi->cinfo.chip_index > 0)
		function_num = bmdi->cinfo.chip_index;
	else
		function_num = bmdi->function_number;

	if (bmdi->boot_info.deadbeef == 0xdeadbeef) {
		switch (board_type) {
		case BOARD_TYPE_EVB:
		case BOARD_TYPE_SC5:
			if (bmdi->boot_info.board_power_sensor_exist != 1 ||
				bmdi->boot_info.fan_exist != 1 ||
				bmdi->boot_info.max_board_power != 30) {

				bmdi->boot_info.board_power_sensor_exist = 1;
				bmdi->boot_info.fan_exist = 1;
				bmdi->boot_info.max_board_power = 30;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SC5_PLUS:
			if (bmdi->boot_info.fan_exist != 0 ||
				bmdi->boot_info.max_board_power != 75) {

				bmdi->boot_info.fan_exist = 0;
				bmdi->boot_info.max_board_power = 75;
				need_update = 1;
			}
			if (function_num == 0x0 &&
				bmdi->boot_info.board_power_sensor_exist != 1) {

				bmdi->boot_info.board_power_sensor_exist = 1;
				need_update = 1;
			}
			if (function_num != 0x0 &&
				bmdi->boot_info.board_power_sensor_exist == 1) {

				bmdi->boot_info.board_power_sensor_exist = 0;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SC5_H:
			if (bmdi->boot_info.fan_exist != 1 ||
				bmdi->boot_info.max_board_power != 30 ||
				bmdi->boot_info.board_power_sensor_exist != 1) {

				bmdi->boot_info.fan_exist = 1;
				bmdi->boot_info.max_board_power = 30;
				bmdi->boot_info.board_power_sensor_exist = 1;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SM5_P:
		case BOARD_TYPE_SM5_S:
			if (bmdi->boot_info.board_power_sensor_exist != 0 ||
				bmdi->boot_info.fan_exist != 0 ||
				bmdi->boot_info.max_board_power != 25) {

				bmdi->boot_info.board_power_sensor_exist = 0;
				bmdi->boot_info.fan_exist = 0;
				bmdi->boot_info.max_board_power = 25;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SC7_PRO:
			if (bmdi->boot_info.board_power_sensor_exist != 1 ||
				bmdi->boot_info.fan_exist != 0 ||
				bmdi->boot_info.max_board_power != 300 ||
				bmdi->boot_info.tpu_max_clk != 875 ||
				bmdi->boot_info.tpu_min_clk != 25) {
				bmdi->boot_info.board_power_sensor_exist = 1;
				bmdi->boot_info.max_board_power = 300;
				bmdi->boot_info.tpu_min_clk = 25;
				bmdi->boot_info.tpu_max_clk = 875;
				bmdi->boot_info.fan_exist = 0;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SC7_FP150:
			if (bmdi->boot_info.board_power_sensor_exist != 1 ||
				bmdi->boot_info.fan_exist != 0 ||
				bmdi->boot_info.max_board_power != 150 ||
				bmdi->boot_info.tpu_max_clk != 800 ||
				bmdi->boot_info.tpu_min_clk != 25) {
				bmdi->boot_info.board_power_sensor_exist = 1;
				bmdi->boot_info.max_board_power = 150;
				bmdi->boot_info.tpu_min_clk = 25;
				bmdi->boot_info.tpu_max_clk = 800;
				bmdi->boot_info.fan_exist = 0;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SC7_PLUS:
			if (bmdi->boot_info.board_power_sensor_exist != 1 ||
				bmdi->boot_info.fan_exist != 0 ||
				bmdi->boot_info.max_board_power != 75 ||
				bmdi->boot_info.tpu_min_clk != 25 ||
				bmdi->boot_info.tpu_max_clk != 750) {
				bmdi->boot_info.board_power_sensor_exist = 1;
				bmdi->boot_info.max_board_power = 75;
				bmdi->boot_info.tpu_min_clk = 25;
				bmdi->boot_info.tpu_max_clk = 750;
				bmdi->boot_info.fan_exist = 0;
				need_update = 1;
			}
			break;
		case BOARD_TYPE_SE5:
		case BOARD_TYPE_SA6:
		case BOARD_TYPE_BM1684X_EVB:
			if (bmdi->boot_info.ddr_rank_mode != 0x02 || bmdi->boot_info.ddr_mode != 0x01){
				bmdi->boot_info.ddr_rank_mode = 0x02;
				bmdi->boot_info.ddr_mode = 0x01;
				need_update = 1;
			}
			break;
		default:
			TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,"unknow board type = %d \n", board_type);
			return -1;
		}
	}

	if (need_update == 0x1)
		ret = bm_spi_flash_update_boot_info(bmdi, &bmdi->boot_info);

	return ret;
}

int bmdrv_set_pld_boot_info(struct bm_device_info *bmdi)
{
	bmdi->boot_info.ddr_rank_mode = 0x3;
	bmdi->boot_info.ddr_1_size = 0x100000000;
	bmdi->boot_info.ddr_2_size = 0x100000000;
	bmdi->boot_info.board_power_sensor_exist = 0;
	bmdi->boot_info.ddr_ecc_enable = 0;
	bmdi->boot_info.ddr_0a_size = 0x80000000;
	bmdi->boot_info.ddr_0b_size = 0x80000000;
	bmdi->boot_info.ddr_mode = 0x1;
	bmdi->boot_info.ddr_vendor_id = 0;
	bmdi->boot_info.fan_exist = 0;
	bmdi->boot_info.tpu_min_clk = 75;
	bmdi->boot_info.tpu_max_clk = 550;
	bmdi->boot_info.max_board_power = 75;
	bmdi->boot_info.temp_sensor_exist = 0;
	bmdi->boot_info.tpu_power_sensor_exist = 0;
	bmdi->boot_info.deadbeef = 0xdeadbeef;
	return 0;
}

static int bmdrv_set_1684x_default_boot_info(struct bm_device_info* bmdi)
{
	int board_version = 0;
	u8 board_type = 0;

	board_version = bmdi->cinfo.board_version;
	board_type = (u8)((board_version >> 8) & 0xff);

	bmdi->boot_info.ddr_rank_mode = 0x2;
	bmdi->boot_info.ddr_1_size = 0x100000000;
	bmdi->boot_info.ddr_2_size = 0x100000000;
	bmdi->boot_info.board_power_sensor_exist = 1;
	bmdi->boot_info.ddr_ecc_enable = 0;
	bmdi->boot_info.ddr_0a_size = 0x100000000;
	bmdi->boot_info.ddr_0b_size = 0x100000000;
	bmdi->boot_info.ddr_mode = 0x1;
	bmdi->boot_info.ddr_vendor_id = 0;
	bmdi->boot_info.fan_exist = 0;
	bmdi->boot_info.tpu_min_clk = 25;
	bmdi->boot_info.tpu_max_clk = 1000;
	bmdi->boot_info.max_board_power = 30;
	bmdi->boot_info.temp_sensor_exist = 1;
	bmdi->boot_info.tpu_power_sensor_exist = 0;
	bmdi->boot_info.deadbeef = 0xdeadbeef;
	bmdi->boot_info.boot_info_version = BOOT_INFO_VERSION_V1;
	bmdi->boot_info.append.append_v1.a53_enable = 0x1;
	bmdi->boot_info.append.append_v1.heap2_size = 0x40000000;
	if (board_type == BOARD_TYPE_SC7_PRO) {
		bmdi->boot_info.max_board_power = 300;
	} else if(board_type == BOARD_TYPE_SC7_FP150) {
		bmdi->boot_info.max_board_power = 150;
	} else if (board_type == BOARD_TYPE_SC7_PLUS) {
		bmdi->boot_info.max_board_power = 75;
	}

	return 0;
}

int bmdrv_boot_info_init(struct bm_device_info *bmdi)
{
	int rc = 0;
	struct chip_info *cinfo = &bmdi->cinfo;

	if (cinfo->platform == PALLADIUM) {
		rc = bmdrv_set_pld_boot_info(bmdi);
		bmdrv_dump_bootinfo(bmdi);
		return rc;
	}

	if (0x1684 == cinfo->chip_id || 0x1686 == cinfo->chip_id) {
		rc = bm_spi_flash_get_boot_info(bmdi, &bmdi->boot_info);
		if (rc) {
            TraceEvents(TRACE_LEVEL_ERROR,
                        TRACE_DRIVER,
                        "read boot info fail %d\n",
                        rc);
			return rc;
		} else {
			if (bmdi->boot_info.deadbeef != 0xdeadbeef) {
                TraceEvents(TRACE_LEVEL_ERROR,
                            TRACE_DRIVER,
                            "invalid boot info, set to default val!\n");
				if (cinfo->chip_id == 0x1684) {
					if (bmdrv_set_default_boot_info(bmdi) != 0)
						return -1;
				}
				else {
					bmdrv_set_1684x_default_boot_info(bmdi);
					TraceEvents(TRACE_LEVEL_INFORMATION,TRACE_DRIVER,"1684x bmdrv_boot_info_init \n");
				}
				rc = bm_spi_flash_update_boot_info(bmdi, &bmdi->boot_info);
			}
		}
		bmdrv_check_bootinfo(bmdi);
		bmdrv_dump_bootinfo(bmdi);
	}
	return rc;
}

