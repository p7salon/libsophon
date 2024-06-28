#include "../bm_common.h"
#include "vpp_platform.h"
#include "vpp_platform.tmh"



int vpp_init(struct bm_device_info *bmdi)
{
  /*if (bmdi->cinfo.chip_id == 0x1684) {
    bm1684_vpp_init(bmdi);
    bmdi->vppdrvctx.vpp_init           = bm1684_vpp_init;
    bmdi->vppdrvctx.vpp_exit           = bm1684_vpp_exit;
    bmdi->vppdrvctx.trigger_vpp        = bm1684_trigger_vpp;
    bmdi->vppdrvctx.bm_vpp_request_irq = bm1684_vpp_request_irq;
    bmdi->vppdrvctx.bm_vpp_free_irq    = bm1684_vpp_free_irq;
  }*/

  if (bmdi->cinfo.chip_id == 0x1686) {
    bm1686_vpp_init(bmdi);
    bmdi->vppdrvctx.vpp_init           = bm1686_vpp_init;
    bmdi->vppdrvctx.vpp_exit           = bm1686_vpp_exit;
    bmdi->vppdrvctx.trigger_vpp        = bm1686_trigger_vpp;
    bmdi->vppdrvctx.bm_vpp_request_irq = bm1686_vpp_request_irq;
    bmdi->vppdrvctx.bm_vpp_free_irq    = bm1686_vpp_free_irq;
  }

  if(NULL == bmdi->vppdrvctx.vpp_init) {
    DbgPrint("vpp_init failed, bmdi->cinfo.chip_id %d\n",bmdi->cinfo.chip_id);
    return -1;
  }

  bm1686_vpp_init(bmdi);
  return 0;
}

void vpp_exit(struct bm_device_info *bmdi)
{
  if(NULL == bmdi->vppdrvctx.vpp_exit) {
    DbgPrint("vpp_exit failed, bmdi->cinfo.chip_id %d\n",bmdi->cinfo.chip_id);
    return;
  }

  bm1686_vpp_exit(bmdi);
  return;
}

int trigger_vpp(struct bm_device_info *bmdi, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t  InputBufferLength)
{
  int ret = 0;

  /*if(NULL == bmdi->vppdrvctx.trigger_vpp) {
      TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_VPP, "trigger_vpp failed, bmdi->cinfo.chip_id %d\n", bmdi->cinfo.chip_id);
    return -1;
  }*/
  ret = bm1686_trigger_vpp(bmdi, Request, OutputBufferLength, InputBufferLength);
  return ret;
}

void bm_vpp_request_irq(struct bm_device_info *bmdi)
{
  if(NULL == bmdi->vppdrvctx.bm_vpp_request_irq) {
    DbgPrint("bm_vpp_request_irqfailed, bmdi->cinfo.chip_id %d\n",bmdi->cinfo.chip_id);
    return;
  }

  bm1686_vpp_request_irq(bmdi);
  return;
}

void bm_vpp_free_irq(struct bm_device_info *bmdi)
{
  if(NULL == bmdi->vppdrvctx.bm_vpp_free_irq) {
    DbgPrint("bm_vpp_free_irq, bmdi->cinfo.chip_id %d\n",bmdi->cinfo.chip_id);
    return;
  }

  bm1686_vpp_free_irq(bmdi);
  return;
}

