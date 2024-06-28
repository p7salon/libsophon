#ifndef _VPP_PLATFORM
#define _VPP_PLATFORM



#define VPP_CORE_MAX (2)
typedef struct vpp_drv_context {
  KSEMAPHORE vpp_core_sem;
  u64 vpp_idle_bit_map;
  KEVENT wq_vpp[VPP_CORE_MAX];
  int got_event_vpp[VPP_CORE_MAX];

  int  (*trigger_vpp)(struct bm_device_info *bmdi, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t  InputBufferLength);
  void (*bm_vpp_request_irq)(struct bm_device_info *bmdi);
  void (*bm_vpp_free_irq)(struct bm_device_info *bmdi);
  int  (*vpp_init)(struct bm_device_info *bmdi);
  void (*vpp_exit)(struct bm_device_info *bmdi);
  WDFSPINLOCK   vpp_spinlock;
} vpp_drv_context_t;


int  trigger_vpp(struct bm_device_info *bmdi, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t  InputBufferLength);
void bm_vpp_request_irq(struct bm_device_info *bmdi);
void bm_vpp_free_irq(struct bm_device_info *bmdi);
int  vpp_init(struct bm_device_info *bmdi);
void vpp_exit(struct bm_device_info *bmdi);



int  bm1684_trigger_vpp(struct bm_device_info *bmdi, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t  InputBufferLength);
void bm1684_vpp_request_irq(struct bm_device_info *bmdi);
void bm1684_vpp_free_irq(struct bm_device_info *bmdi);
int  bm1684_vpp_init(struct bm_device_info *bmdi);
void bm1684_vpp_exit(struct bm_device_info *bmdi);

int  bm1686_trigger_vpp(struct bm_device_info *bmdi, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t  InputBufferLength);
void bm1686_vpp_request_irq(struct bm_device_info *bmdi);
void bm1686_vpp_free_irq(struct bm_device_info *bmdi);
int  bm1686_vpp_init(struct bm_device_info *bmdi);
void bm1686_vpp_exit(struct bm_device_info *bmdi);

#endif