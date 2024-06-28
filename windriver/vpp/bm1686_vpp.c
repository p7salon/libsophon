#include "../bm_common.h"
#include "bm1686_vpp.tmh"

#define VPP_OK                             (0)
#define VPP_ERR                            (-1)
#define VPP_ERR_COPY_FROM_USER             (-2)
#define VPP_ERR_WRONG_CROPNUM              (-3)
#define VPP_ERR_INVALID_FD                 (-4)
#define VPP_ERR_INT_TIMEOUT                (-5)
#define VPP_ERR_INVALID_PA                 (-6)
#define VPP_ERR_INVALID_CMD                (-7)

#define VPP_ENOMEM                         (-12)
#define VPP_ERR_IDLE_BIT_MAP               (-256)
#define VPP_ERESTARTSYS                    (-512)

static struct vpp_reset_info bm_vpp_rst[VPP1686_CORE_MAX] = {
	{SW_RESET_REG1, VPP_CORE0_RST},
	{SW_RESET_REG2, VPP_CORE1_RST},
};


#if 0
/*Positive number * 1024*/
/*negative number * 1024, then Complement code*/
YPbPr2RGB, BT601
Y = 0.299 R + 0.587 G + 0.114 B
U = -0.1687 R - 0.3313 G + 0.5 B + 128
V = 0.5 R - 0.4187 G - 0.0813 B + 128
R = Y + 1.4018863751529200 (Cr-128)
G = Y - 0.345806672214672 (Cb-128) - 0.714902851111154 (Cr-128)
B = Y + 1.77098255404941 (Cb-128)

YCbCr2RGB, BT601
Y = 16  + 0.257 * R + 0.504 * g + 0.098 * b
Cb = 128 - 0.148 * R - 0.291 * g + 0.439 * b
Cr = 128 + 0.439 * R - 0.368 * g - 0.071 * b
R = 1.164 * (Y - 16) + 1.596 * (Cr - 128)
G = 1.164 * (Y - 16) - 0.392 * (Cb - 128) - 0.812 * (Cr - 128)
B = 1.164 * (Y - 16) + 2.016 * (Cb - 128)
#endif


static void vpp_reg_write(s8 core_id, struct bm_device_info *bmdi, unsigned int val, unsigned int offset)
{
	if (core_id == 0)
		vpp0_reg_write(bmdi, offset, val);
	else
		vpp1_reg_write(bmdi, offset, val);
}

static unsigned int vpp_reg_read(s8 core_id, struct bm_device_info *bmdi, unsigned int offset)
{
	unsigned int ret;

	if (core_id == 0)
		ret = vpp0_reg_read(bmdi, offset);
	else
		ret = vpp1_reg_read(bmdi, offset);
	return ret;
}
#if  1
static int vpp_reg_dump(struct bm_device_info *bmdi,s8 core_id)
{
	u32 reg_value = 0;

	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "core_id: %d\n",core_id);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_VERSION);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_VERSION: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_CONTROL0);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_CONTROL0: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_CMD_BASE);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_CMD_BASE: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_CMD_BASE_EXT);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_CMD_BASE_EXT: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_STATUS);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_STATUS: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_INT_EN);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_INT_EN: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_INT_CLEAR);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_INT_CLEAR: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_INT_STATUS);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_INT_STATUS: 0x%x\n",reg_value);
	reg_value = vpp_reg_read(core_id, bmdi, BM1686_VPP_INT_RAW_STATUS);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "VPP_INT_RAW_STATUS: 0x%x\n",reg_value);

	return 0;
}

static int top_reg_dump(struct bm_device_info *bmdi)
{

	u32 reg_value = 0;

	reg_value = bm_read32(bmdi,SW_RESET_REG0);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "SW_RESET_REG0: 0x%x\n",reg_value);
	reg_value = bm_read32(bmdi,SW_RESET_REG1);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "SW_RESET_REG1: 0x%x\n",reg_value);
	reg_value = bm_read32(bmdi,SW_RESET_REG2);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "SW_RESET_REG2: 0x%x\n",reg_value);

	return 0;
}
static int vpp_regs_dump(struct bm_device_info *bmdi)
{
	s8 core_id;

	top_reg_dump(bmdi);

	for (core_id = 0; core_id < VPP1686_CORE_MAX; core_id++)
		vpp_reg_dump(bmdi, core_id);

	return 0;
}

static int vpp_core_reset(struct bm_device_info *bmdi, s8 core_id)
{
	u32 val = 0;

/*add mutex lock*/
	val = bm_read32(bmdi, bm_vpp_rst[core_id].reg);
	val &= ~(1 << bm_vpp_rst[core_id].bit_n);
	bm_write32(bmdi, bm_vpp_rst[core_id].reg, val);
	bm_udelay(10);
	val |= (1 << bm_vpp_rst[core_id].bit_n);
	bm_write32(bmdi, bm_vpp_rst[core_id].reg, val);
/*add mutex unlock*/

	return 0;
}
#endif
static int vpp_soft_rst(struct bm_device_info *bmdi, s8 core_id)
{
#if 0
	u32 reg_read = 0;
	u32 active_check = 0;
	u32 count_value = 0;

	/*set dma_stop=1*/
	vpp_reg_write(core_id, bmdi, 0x104, VPP_CONTROL0);

	/*check dma_stop_active==1*/
	for (count_value = 0; count_value < 1000; count_value++)
	{
		reg_read = vpp_reg_read(core_id, bmdi, VPP_STATUS);
		if ((reg_read >> 17) & 0x1)
		{
			active_check++;
			if (active_check == 5)
			{
				break;
			}
		}
		bm_udelay(1);
	}
	if((1000 == count_value) && (5 != active_check))
	{
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV] dma_stop failed, continue vpp soft reset\n");
	}
#endif
	/*vpp soft reset*/
	vpp_reg_write(core_id, bmdi, 0x80, BM1686_VPP_CONTROL0);

	bm_udelay(10);

	vpp_reg_write(core_id, bmdi, 0x82, BM1686_VPP_CONTROL0);

#if 0
/*deassert dma_stop and check dma_stop_active*/
	vpp_reg_write(core_id, bmdi, 0x100, VPP_CONTROL0);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "vpp_soft_rst done\n");
#endif

	return VPP_OK;
}

static void vpp_clear_int(struct bm_device_info *bmdi, s8 core_id)
{
	if (core_id == 0)
		vpp0_reg_write(bmdi, BM1686_VPP_INT_CLEAR, 0xffffffff);
	else
		vpp1_reg_write(bmdi, BM1686_VPP_INT_CLEAR, 0xffffffff);
}

static void vpp_irq_handler(struct bm_device_info *bmdi, s8 core_id)
{
	KeSetEvent(&bmdi->vppdrvctx.wq_vpp[core_id], 0, FALSE);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "vpp irq handler, core id %d\n", core_id);
}

static int vpp_check_hw_idle(s8 core_id, struct bm_device_info *bmdi)
{
	unsigned int status, val;
	unsigned int count = 0;

	while (1) {
		status = vpp_reg_read(core_id, bmdi, BM1686_VPP_STATUS);
		val = (status >> 16) & 0x1;
		if (val) // idle
			break;

		count++;
		if (count > 20000) { // 2 Sec
			TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV]vpp is busy!!! status 0x%08x, core %d, device %d\n",
			       status, core_id, bmdi->dev_index);
			vpp_soft_rst(bmdi,core_id);
			return VPP_ERR;
		}
		bm_udelay(100);
	}

	count = 0;
	while (1) {
		status = vpp_reg_read(core_id, bmdi, BM1686_VPP_INT_RAW_STATUS);
		if (status == 0x0)
			break;

		count++;
		if (count > 20000) { // 2 Sec
			TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "vpp raw status 0x%08x, core %d, device %d\n",
			       status, core_id, bmdi->dev_index);
			vpp_soft_rst(bmdi,core_id);
			return VPP_ERR;
		}
		bm_udelay(100);

		vpp_reg_write(core_id, bmdi, 0xffffffff, BM1686_VPP_INT_CLEAR);
	}

	return VPP_OK;
}

static int vpp_setup_desc(struct bm_device_info *bmdi, struct vpp_batch_n *batch, s8 core_id)
{
	int ret;
	uint32 reg_value = 0;
	u64 vpp_desc_pa = 0;
	unsigned char vpp_id_mode = Transaction_Mode;
	UNREFERENCED_PARAMETER(batch);

	vpp_soft_rst(bmdi,core_id);

	/* check vpp hw idle */
	ret = vpp_check_hw_idle(core_id, bmdi);
	if (ret < 0) {
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV]vpp_check_hw_idle failed! core_id %d, dev_index %d.\n", core_id, bmdi->dev_index);
		return ret;
	}

	/*set cmd list addr*/
	vpp_desc_pa = bmdi->gmem_info.resmem_info.vpp_addr + core_id * (512 << 10);
	vpp_reg_write(core_id, bmdi, (unsigned int)(vpp_desc_pa & 0xffffffff), BM1686_VPP_CMD_BASE);
	reg_value = ((vpp_id_mode & 0x3) << 29 ) | ((vpp_desc_pa >> 32) & 0xff);// vpp_id_mode;

	vpp_reg_write(core_id, bmdi, reg_value, BM1686_VPP_CMD_BASE_EXT);
	vpp_reg_write(core_id, bmdi, 0x01, BM1686_VPP_INT_EN);//interruput mode : frame_done_int_en

	/*start vpp hw work*/
	vpp_reg_write(core_id, bmdi, 0x41, BM1686_VPP_CONTROL0);

	return VPP_OK;
}

static int vpp_prepare_cmd_list(struct bm_device_info *bmdi,
				struct vpp_batch_n *batch,
				s8 core_id)
{
	int idx, ret=0;
	u64 vpp_desc_pa=0;

	for (idx = 0; idx < batch->num; idx++) {

		descriptor *pdes = (batch->cmd + idx);
		if (((pdes->src_base_ext.src_base_ext_ch0 & 0xff) != 0x1) &&
		  ((pdes->src_base_ext.src_base_ext_ch0 & 0xff) != 0x2) &&
		  ((pdes->src_base_ext.src_base_ext_ch0 & 0xff) != 0x3) &&
		  ((pdes->src_base_ext.src_base_ext_ch0 & 0xff) != 0x4))
		{
			TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV]pdes->src_base_ext.src_base_ext_ch0  0x%x\n", pdes->src_base_ext.src_base_ext_ch0);
			ret = VPP_ERR_INVALID_PA;
			return ret;
		}

		vpp_desc_pa = bmdi->gmem_info.resmem_info.vpp_addr + core_id * (512 << 10) + sizeof(descriptor) * (idx + 1);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "sizeof: %d\n", sizeof(descriptor));

		if (idx == (batch->num - 1)) {
			pdes->des_head.crop_flag = 1;
			pdes->des_head.next_cmd_base_ext = 0;
			pdes->next_cmd_base= 0;
		}
		else {
			pdes->des_head.crop_flag = 0;
			pdes->des_head.next_cmd_base_ext = (uint32)((vpp_desc_pa >> 32) & 0xff);
			pdes->next_cmd_base= (uint32)(vpp_desc_pa & 0xffffffff);
		}

		pdes->des_head.crop_id = idx;
	}
	//dump_des(batch, pdes, vpp_desc_pa);

	vpp_desc_pa = bmdi->gmem_info.resmem_info.vpp_addr + core_id * (512 << 10);

	ret = bmdev_memcpy_s2d_internal(bmdi, vpp_desc_pa, (void *)(batch->cmd), batch->num * sizeof(descriptor));
	if (ret != VPP_OK) {
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV]bmdev_memcpy_s2d_internal failed, dev_index %d\n", bmdi->dev_index);
		return ret;
	}

	return ret;
}

static int vpp_get_core_id(struct bm_device_info *bmdi, s8 *core)
{
	if (bmdi->vppdrvctx.vpp_idle_bit_map >= 3) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "[fatal err!]take sem, but two vpp core are busy, vpp_idle_bit_map = %lld\n", bmdi->vppdrvctx.vpp_idle_bit_map);
		return VPP_ERR_IDLE_BIT_MAP;
	}

	WdfSpinLockAcquire(bmdi->vppdrvctx.vpp_spinlock);

	if (0x0 == (bmdi->vppdrvctx.vpp_idle_bit_map & 0x1)) {
		*core = 0;
		bmdi->vppdrvctx.vpp_idle_bit_map = (bmdi->vppdrvctx.vpp_idle_bit_map | 0x1);
	}
	else if (0x0 == (bmdi->vppdrvctx.vpp_idle_bit_map & 0x2)) {
		*core = 1;
		bmdi->vppdrvctx.vpp_idle_bit_map = (bmdi->vppdrvctx.vpp_idle_bit_map | 0x2);
	}
	else {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "[fatal err!]Abnormal status, vpp_idle_bit_map = %lld\n", bmdi->vppdrvctx.vpp_idle_bit_map);
		WdfSpinLockRelease(bmdi->vppdrvctx.vpp_spinlock);
		return VPP_ERR_IDLE_BIT_MAP;
	}
	WdfSpinLockRelease(bmdi->vppdrvctx.vpp_spinlock);

	return VPP_OK;
}

static int vpp_free_core_id(struct bm_device_info *bmdi, s8 core_id)
{
	if ((core_id != 0) && (core_id != 1)) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "vpp abnormal status, vpp_idle_bit_map = %lld, core_id is %d\n", bmdi->vppdrvctx.vpp_idle_bit_map, core_id);
		return VPP_ERR_IDLE_BIT_MAP;
	}

	WdfSpinLockAcquire(bmdi->vppdrvctx.vpp_spinlock);
	bmdi->vppdrvctx.vpp_idle_bit_map = (bmdi->vppdrvctx.vpp_idle_bit_map & (~(0x1 << core_id)));
	WdfSpinLockRelease(bmdi->vppdrvctx.vpp_spinlock);

	return VPP_OK;
}
#if 0
static void dump_des(struct vpp_batch_n *batch, struct vpp1686_descriptor_s **pdes_vpp, dma_addr_t *des_paddr)
{
	int idx = 0;

	//TraceEvents("bmdi->dev_index   is      %d\n", bmdi->dev_index);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "batch->num   is      %d\n", batch->num);
	for (idx = 0; idx < batch->num; idx++) {
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "des_paddr[%d]   0x%llx\n", idx, des_paddr[idx]);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "pdes_vpp[%d]->des_head   crop_id: 0x%x\n", idx, pdes_vpp[idx]->des_head.crop_id);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "pdes_vpp[%d]->next_cmd_base   0x%x\n", idx, pdes_vpp[idx]->next_cmd_base);
	}
}

void vpp_dump(struct vpp_batch_n *batch)
{
	struct vpp1686_descriptor_s *cmd ;
	int i;
	for (i = 0; i < batch->num; i++) {
		cmd = (batch->cmd + i);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "batch->num    is  %d      \n", batch->num);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "cmd id %d, cmd->src_format  height: 0x%x\n", i, cmd->src_size.src_height);
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "cmd id %d, cmd->src_stride  0x%x\n", i, cmd->src_stride_ch0.src_stride_ch0);
	}
	return;
}
#endif

s32 bm1686_vpp_handle_setup(struct bm_device_info *bmdi, struct vpp_batch_n *batch)
{
	s32 ret = VPP_OK;
	s8 core_id = -1;
	NTSTATUS  status;
	LARGE_INTEGER li;

	li.QuadPart = WDF_REL_TIMEOUT_IN_MS(10000);
	struct reserved_mem_info *resmem_info = &bmdi->gmem_info.resmem_info;

	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV]start vpp_handle_setup\n");

	u64 count = KeReadStateSemaphore(&bmdi->vppdrvctx.vpp_core_sem);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "The Semaphore count is %lld\n", count);
	KeWaitForSingleObject(&bmdi->vppdrvctx.vpp_core_sem, Executive, KernelMode, FALSE, NULL);

	ret = vpp_get_core_id(bmdi, &core_id);
	if (ret != VPP_OK)
		goto err1;

	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "resmem_info->vpp_addr: 0x%llx, core_id %d", resmem_info->vpp_addr, core_id);

	ret = vpp_prepare_cmd_list(bmdi, batch, core_id);
	if (ret != VPP_OK) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "vpp_prepare_cmd_list failed \n");
		goto  err3;
	}

	ret = vpp_setup_desc(bmdi, batch, core_id);
	if (ret != VPP_OK) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "vpp_setup_desc failed \n");
		goto  err3;
	}

	status = KeWaitForSingleObject(&bmdi->vppdrvctx.wq_vpp[core_id], Executive, KernelMode, FALSE, &li);
	if (status == STATUS_TIMEOUT) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "vpp timeout,status  %d,core_id %d,vpp_idle_bit_map %lld, dev_index  %d\n",
		status, core_id, bmdi->vppdrvctx.vpp_idle_bit_map, bmdi->dev_index);
		vpp_reg_dump(bmdi, core_id);
		vpp_soft_rst(bmdi, core_id);
		ret = VPP_ERR_INT_TIMEOUT;
	}
	if ((status == STATUS_ALERTED) || (status == STATUS_USER_APC))
	{
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "vpp signal_pending \n");
	}

	if (status == STATUS_SUCCESS)
	{
		TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "vpp run success \n");
	}

err3:

//err2:
    ret |= vpp_free_core_id(bmdi, core_id);

err1:
	KeReleaseSemaphore(&bmdi->vppdrvctx.vpp_core_sem, 0, 1, FALSE);

	return ret;
}



int bm1686_trigger_vpp(struct bm_device_info *bmdi, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t  InputBufferLength)
{
	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);
	struct vpp_batch_n *batch;

	NTSTATUS Status = 0;
	size_t bufSize = 0;

	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "[1686VPPDRV] start trigger_vpp\n");

	Status = WdfRequestRetrieveInputBuffer(Request, sizeof(struct vpp_batch_n), &batch, &bufSize);
	if (!NT_SUCCESS(Status)) {
		WdfRequestCompleteWithInformation(Request, Status, 0);
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "trigger_vpp copy_from_user wrong: "
			"%d, sizeof(struct vpp_batch) total need %u, really copyied %llu,dev_index  %d\n",
			Status, sizeof(struct vpp_batch_n), bufSize, bmdi->dev_index);
		Status = VPP_ERR_COPY_FROM_USER;
		return Status;
	}
	if ((batch->num <= 0) || (batch->num > VPP_CROP_NUM_MAX)) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "wrong num, batch.num  %d, dev_index  %d\n", batch->num, bmdi->dev_index);
		Status = VPP_ERR_WRONG_CROPNUM;
		WdfRequestCompleteWithInformation(Request, STATUS_INVALID_PARAMETER, 0);
		return Status;
	}
	Status = bm1686_vpp_handle_setup(bmdi, batch);
	if ((Status != VPP_OK) && (Status != VPP_ERESTARTSYS)) {
		TraceEvents(TRACE_LEVEL_ERROR, TRACE_VPP, "trigger_vpp ,vpp_handle_setup wrong, ret %d, line  %!LINE!, "
			"dev_index  %d\n", Status, bmdi->dev_index);
		WdfRequestCompleteWithInformation(Request, STATUS_UNSUCCESSFUL, 0);
	}
	else {
		WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, 0);
	}
	return Status;
}

int bm1686_vpp_init(struct bm_device_info *bmdi)
{
	s32 i;
	WDF_OBJECT_ATTRIBUTES	attributes;
	NTSTATUS				status;

	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "%!FUNC! enter\n");

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = bmdi->WdfDevice;
	status = WdfSpinLockCreate(&attributes, &bmdi->vppdrvctx.vpp_spinlock);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	KeInitializeSemaphore(&bmdi->vppdrvctx.vpp_core_sem, VPP_CORE_MAX, VPP_CORE_MAX);
	bmdi->vppdrvctx.vpp_idle_bit_map = 0;
	for (i = 0; i < VPP_CORE_MAX; i++)
		KeInitializeEvent(&bmdi->vppdrvctx.wq_vpp[i], SynchronizationEvent, FALSE);
	TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "%!FUNC! exit\n");

	return 0;
}

void bm1686_vpp_exit(struct bm_device_info *bmdi)
{
	UNREFERENCED_PARAMETER(bmdi);
}

static void bmdrv_vpp0_irq_handler(struct bm_device_info *bmdi)
{
	vpp_clear_int(bmdi, 0);
	vpp_irq_handler(bmdi,0);
}

static void bmdrv_vpp1_irq_handler(struct bm_device_info *bmdi)
{
	vpp_clear_int(bmdi, 1);
	vpp_irq_handler(bmdi, 1);
}

void bm1686_vpp_request_irq(struct bm_device_info *bmdi)
{
	bmdrv_submodule_request_irq(bmdi, VPP0_IRQ_ID, bmdrv_vpp0_irq_handler);
	bmdrv_submodule_request_irq(bmdi, VPP1_IRQ_ID, bmdrv_vpp1_irq_handler);
}

void bm1686_vpp_free_irq(struct bm_device_info *bmdi)
{
	bmdrv_submodule_free_irq(bmdi, VPP0_IRQ_ID);
	bmdrv_submodule_free_irq(bmdi, VPP1_IRQ_ID);
}
