#ifndef __GST_CUSTOMVENC_H__
#define __GST_CUSTOMVENC_H__

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideoencoder.h>
#include <list.h>
#include <stdint.h>

/*
#include <video encoder 库的头文件>
 */

G_BEGIN_DECLS

#define GST_TYPE_CUSTOMVENC \
  (gst_customvenc_get_type())
#define GST_CUSTOMVENC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_CUSTOMVENC,GstCustomVEnc))
#define GST_CUSTOMVENC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_CUSTOMVENC,GstCustomVEncClass))
#define GST_IS_CUSTOMVENC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_CUSTOMVENC))
#define GST_IS_CUSTOMVENC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_CUSTOMVENC))

typedef struct _GstCustomVEnc      GstCustomEnc;
typedef struct _GstCustomVEncClass GstCustomEncClass;
typedef struct _GstCustomVEncVTable GstCustomEncVTable;

struct _GstCustomVEnc
{
  GstVideoEncoder element;

  /* internal codec related */
  struct internal_codec_info {
    venc_t handle;  // video encoder 句柄
    venc_id id;     // codec id 指明是H264还是H265编码
    guchar *buf;    // 编码后的输出缓存
  } codec;

  GstAllocator *dma_buffer;

  /* properties */
  gint gop;
  gint framerate;
  guint bitrate;
  guint min_buffers;
  guint max_buffers;
  guint buffsize_of_enc;

  struct roi_info {
    guint source_id;
    gboolean is_enabled;
    gint current_id;
    gint size_of_block;
    struct listnode param_info;
    struct _buffer {
      gint width;
      gint height;
      guchar *data;
    } buffer_info;
  } roi;

  /* used to get input state */
  GstVideoCodecState *input_state;

};

struct _GstCustomVEncClass
{
  GstVideoEncoderClass parent_class;
};

GType gst_customvenc_get_type (void);

G_END_DECLS

#endif /* __GST_CUSTOMVENC_H__ */
