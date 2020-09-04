/**
 * SECTION:element-customvenc
 *
 * FIXME:Describe customvenc here.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v -m fakesrc ! customvenc ! fakesink silent=TRUE
 * ]|
 * </refsect2>
 */

#include <gmodule.h>
#include <gst/allocators/gstamlionallocator.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/gst.h>
#include <gst/pbutils/pbutils.h>
#include <gst/video/gstvideometa.h>
#include <gst/video/gstvideopool.h>
#include <gst/video/gstvideosink.h>
#include <gst/video/video.h>
#include <string.h>

#include "gstcustomvenc.h"

GST_DEBUG_CATEGORY_STATIC (gst_customvenc_debug);
#define GST_CAT_DEFAULT gst_customvenc_debug

#define P_IDR_PERIOD_DEFAULT 30
#define P_FRAMERATE_DEFAULT 30
#define P_BITRATE_DEFAULT 2000
#define P_BITRATE_MAX 12000
#define P_MIN_BUFFERS_DEFAULT 4
#define P_MAX_BUFFERS_DEFAULT 6
#define P_ENCODER_BUFFER_SIZE_DEFAULT 2048
#define P_ENCODER_BUFFER_SIZE_MIN 1024
#define P_ENCODER_BUFFER_SIZE_MAX 4096

#define P_ROI_ID_DEFAULT 0
#define P_ROI_ENABLED_DEFAULT TRUE
#define P_ROI_WIDTH_DEFAULT 0.00
#define P_ROI_HEIGHT_DEFAULT 0.00
#define P_ROI_X_DEFAULT 0.00
#define P_ROI_Y_DEFAULT 0.00
#define P_ROI_QUALITY_DEFAULT 51

enum
{
  P_0,
  P_GOP,
  P_FRAMERATE,
  P_BITRATE,
  P_MIN_BUFFERS,
  P_MAX_BUFFERS,
  P_ENCODER_BUFSIZE,
  P_ROI_ID,
  P_ROI_ENABLED,
  P_ROI_WIDTH,
  P_ROI_HEIGHT,
  P_ROI_X,
  P_ROI_Y,
  P_ROI_QUALITY,
};


#define PADS_INFO \
        " baseline, constrained-baseline, high-4:4:4-intra, high-4:2:2-intra," \
        "profile = (string) { high-4:4:4, high-4:2:2, high-10, high, main," \
        "framerate = (fraction) [0/1, MAX], " \
        "stream-format = (string) { byte-stream }, " \
        "width = (int) [ 1, MAX ], " "height = (int) [ 1, MAX ], " \
        "alignment = (string) au, " \
        " high-10-intra }"

static GstStaticPadTemplate source_pad = GST_STATIC_PAD_TEMPLATE (
            "src", GST_PAD_SRC, GST_PAD_ALWAYS,
            GST_STATIC_CAPS ("video/x-h264, "
            PADS_INFO "; " "video/x-h265, "PADS_INFO)
            );

static void gst_customvenc_finalize (GObject * obj);
static gboolean gst_customvenc_start (GstVideoEncoder * enc);
static gboolean gst_customvenc_stop (GstVideoEncoder * enc);
static gboolean gst_customvenc_flush (GstVideoEncoder * enc);

static gboolean gst_customvenc_init_enc (GstCustomEnc * encoder);
static void gst_customvenc_close_enc (GstCustomEnc * encoder);

static GstFlowReturn gst_customvenc_handle_frame (GstVideoEncoder * enc,
    GstVideoCodecFrame * frame);
static GstFlowReturn gst_customvenc_encode_frame (GstCustomEnc * enc,
    GstVideoCodecFrame * frame);
static gboolean gst_customvenc_set_format (GstVideoEncoder * video_enc,
    GstVideoCodecState * state);

static void gst_customvenc_set_property (GObject * obj, guint id,
    const GValue * value, GParamSpec * spec);
static void gst_customvenc_get_property (GObject * obj, guint id,
    GValue * value, GParamSpec * spec);

#define gst_customvenc_parent_class parent_class
G_DEFINE_TYPE_WITH_CODE (GstCustomEnc, gst_customvenc, GST_TYPE_VIDEO_ENCODER,
    G_IMPLEMENT_INTERFACE (GST_TYPE_PRESET, NULL));

static venc_image_fmt
convert_image_format (GstVideoFormat vfmt) {
  venc_image_fmt fmt;
  switch (vfmt) {
  case GST_VIDEO_FORMAT_NV12:
    fmt = IMG_FMT_NV12;
    break;
  case GST_VIDEO_FORMAT_NV21:
    fmt = IMG_FMT_NV21;
    break;
  case GST_VIDEO_FORMAT_I420:
  case GST_VIDEO_FORMAT_YV12:
    fmt = IMG_FMT_YUV420P;
    break;
  case GST_VIDEO_FORMAT_RGB:
  case GST_VIDEO_FORMAT_BGR:
    fmt = IMG_FMT_NV12;
    break;
  default:
    fmt = IMG_FMT_NONE;
    break;
  }
  return fmt;
}

static gboolean
gst_customvenc_sink_query (GstVideoEncoder * enc, GstQuery * query)
{
    GstPad *pad = GST_VIDEO_ENCODER_SINK_PAD (enc);
    gboolean ret = FALSE;

    switch (GST_QUERY_TYPE (query)) {
        case GST_QUERY_ACCEPT_CAPS:{
            GstCaps *suported, *caps;
            suported = gst_pad_get_pad_template_caps (pad);
            gst_query_parse_accept_caps (query, &caps);
            gst_query_set_accept_caps_result (query,
                gst_caps_is_subset (caps, suported));
            gst_caps_unref (suported);
            ret = TRUE;
        }
        break;
        default:
            ret = GST_VIDEO_ENCODER_CLASS (parent_class)->sink_query (enc, query);
            break;
    }
    return ret;
}


static void
gst_customvenc_class_init (GstCustomEncClass * klass)
{
    GObjectClass *obj_class;
    GstElementClass *ele_class;
    GstVideoEncoderClass *enc_class;
    GstPadTemplate *sink_templ;
    GstCaps *supported_sinkcaps;

    obj_class = G_OBJECT_CLASS (klass);
    ele_class = GST_ELEMENT_CLASS (klass);
    enc_class = GST_VIDEO_ENCODER_CLASS (klass);

    obj_class->set_property = gst_customvenc_set_property;
    obj_class->get_property = gst_customvenc_get_property;
    obj_class->finalize = gst_customvenc_finalize;

    enc_class->set_format = GST_DEBUG_FUNCPTR (gst_customvenc_set_format);
    enc_class->handle_frame =
        GST_DEBUG_FUNCPTR (gst_customvenc_handle_frame);
    enc_class->start = GST_DEBUG_FUNCPTR (gst_customvenc_start);
    enc_class->stop = GST_DEBUG_FUNCPTR (gst_customvenc_stop);
    enc_class->flush = GST_DEBUG_FUNCPTR (gst_customvenc_flush);
    enc_class->sink_query = GST_DEBUG_FUNCPTR (gst_customvenc_sink_query);

    g_obj_class_install_property (obj_class, P_GOP,
        g_param_spec_int ("gop", "GOP", "key frame interval",
            -1, 1000, P_IDR_PERIOD_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_FRAMERATE,
        g_param_spec_int ("framerate", "Framerate", "framerate(fps)",
            0, 30, P_FRAMERATE_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_BITRATE,
        g_param_spec_int ("bitrate", "Bitrate", "bitrate(kbps)",
            0, P_BITRATE_MAX, P_BITRATE_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_MIN_BUFFERS,
        g_param_spec_int ("min-buffers", "Min-Buffers", "minimum input buffer",
            3, 10, P_MIN_BUFFERS_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_MAX_BUFFERS,
        g_param_spec_int ("max-buffers", "Max-Buffers", "maximum input buffer",
            3, 10, P_MAX_BUFFERS_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_ENCODER_BUFSIZE,
        g_param_spec_int ("enc-buffer-size", "Encoder-Buffer-Size", "Encoder Buffer Size(KBytes)",
            P_ENCODER_BUFFER_SIZE_MIN, P_ENCODER_BUFFER_SIZE_MAX, P_ENCODER_BUFFER_SIZE_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    gst_ele_class_set_static_metadata (
          ele_class, "Customized Video Encoder", "Video Encoder",
          "Customized Video Encoder Plugin", "HenryLee <henry_1_lee@163.com>");

    supported_sinkcaps = gst_caps_new_simple ("video/x-raw",
        "framerate", GST_TYPE_FRACTION_RANGE, 0, 1, G_MAXINT, 1,
        "width", GST_TYPE_INT_RANGE, 16, G_MAXINT,
        "height", GST_TYPE_INT_RANGE, 16, G_MAXINT, NULL);

    sink_templ = gst_pad_template_new ("sink",
        GST_PAD_SINK, GST_PAD_ALWAYS, supported_sinkcaps);

    gst_caps_unref (supported_sinkcaps);

    gst_ele_class_add_pad_template (element_class, sink_templ);
    gst_ele_class_add_static_pad_template (element_class, &source_pad);
}

/* initialize the new element
 * instantiate pads and add them to element
 * set functions
 * initialize structure
 */
static void
gst_customvenc_init (GstCustomEnc * enc)
{
  enc->gop = P_IDR_PERIOD_DEFAULT;
  enc->framerate = P_FRAMERATE_DEFAULT;
  enc->bitrate = P_BITRATE_DEFAULT;
  enc->max_buffers = P_MAX_BUFFERS_DEFAULT;
  enc->min_buffers = P_MIN_BUFFERS_DEFAULT;
  enc->encoder_bufsize = P_ENCODER_BUFFER_SIZE_DEFAULT * 1024;
  enc->codec.id = CODEC_ID_NONE;
}

static void
gst_customvenc_finalize (GObject * obj)
{
  // 当对象的引用计数为0时, 调用父类的finalize函数
  G_OBJECT_CLASS (parent_class)->finalize (obj);
}


/* start 函数, 当开始处理pipeline上的stream流时, 该函数
 * 会被执行, 这里主要对编码后的buffer进行分配
 */
static gboolean
gst_customvenc_start (GstVideoEncoder * enc)
{
  GstCustomEnc *venc = GST_CUSTOMVENC (enc);

  if (venc->codec.buf == NULL) {
    venc->codec.buf = g_new (guchar, venc->enc_bufsize);
  }

  return TRUE;
}

/* stop 函数, 当pipeline上无待处理的stream流时, 该函数
 * 会被执行, 释放相关内存
 */
static gboolean
gst_customvenc_stop (GstVideoEncoder * enc)
{
  GstCustomEnc *venc = GST_CUSTOMVENC (enc);

  gst_customvenc_close_enc (venc);

  if (venc->input_state) {
    gst_video_codec_state_unref (venc->input_state);
    venc->input_state = NULL;
  }

  if (venc->codec.buf) {
    g_free((gpointer)venc->codec.buf);
    venc->codec.buf = NULL;
  }

  if (venc->dmabuf_alloc) {
    gst_obj_unref(venc->dmabuf_alloc);
    venc->dmabuf_alloc = NULL;
  }

  if (venc->imgproc.input.memory) {
    gst_memory_unref(venc->imgproc.input.memory);
    venc->imgproc.input.memory = NULL;
  }

  if (venc->imgproc.output.memory) {
    gst_memory_unref(venc->imgproc.output.memory);
    venc->imgproc.output.memory = NULL;
  }

  return TRUE;
}

// 刷新流
static gboolean
gst_customvenc_flush (GstVideoEncoder * enc)
{
  GstCustomEnc *venc = GST_CUSTOMVENC (enc);

  gst_customvenc_init_enc (venc);

  return TRUE;
}

/*
 * gst_customvenc_init_enc()
 * @encoder: 初始化video encoder
 */
static gboolean
gst_customvenc_init_enc (GstCustomEnc * encoder)
{
  GstVideoInfo *info;

  if (!enc->input_state) {
    GST_DEBUG_OBJECT (enc, "Have no input state yet");
    return FALSE;
  }

  info = &enc->input_state->info;

  /* make sure that the enc is closed */
  gst_customvenc_close_enc (encoder);

  GST_OBJECT_LOCK (enc);


  GST_OBJECT_UNLOCK (enc);

  /* 初始化video encoder */
  /* 此部分使用内部接口,这里不展示 */
  return TRUE;
}


/* gst_customvenc_close_enc
 * @enc:  Encoder which should close.
 *
 * Close v enc.
 */
static void
gst_customvenc_close_enc (GstCustomEnc * encoder)
{
  if (enc->codec.handle != 0) {
    /* 调用encoder lib库接口销毁encoder句柄 */
    vl_multi_enc_destroy(encoder->codec.handle);
    enc->codec.handle = 0;
  }
}

static gboolean
gst_customvenc_set_format (GstVideoEncoder * video_enc,
    GstVideoCodecState * state)
{
  GstCustomEnc *enc = GST_CUSTOMVENC (video_enc);
  GstVideoInfo *info = &state->info;
  GstCaps *template_caps;
  GstCaps *allowed_caps = NULL;
  const gchar* allowed_mime_name = NULL;

  // 如果enc已经初始化,直接使用即可
  if (enc->codec.handle) {
    GstVideoInfo *old = &enc->input_state->info;

    if (info->finfo->format == old->finfo->format
        && info->width == old->width && info->height == old->height
        && info->fps_n == old->fps_n && info->fps_d == old->fps_d
        && info->par_n == old->par_n && info->par_d == old->par_d) {
      gst_video_codec_state_unref (enc->input_state);
      enc->input_state = gst_video_codec_state_ref (state);
      return TRUE;
    }
  }

  if (enc->input_state)
    gst_video_codec_state_unref (enc->input_state);

  enc->input_state = gst_video_codec_state_ref (state);

  template_caps = gst_static_pad_template_get_caps (&source_pad);
  allowed_caps = gst_pad_get_allowed_caps (GST_VIDEO_ENCODER_SRC_PAD (enc));

  if (allowed_caps && allowed_caps != template_caps && enc->codec.id == CODEC_ID_NONE) {
    GstStructure *s;

    if (gst_caps_is_empty (allowed_caps)) {
      gst_caps_unref (allowed_caps);
      gst_caps_unref (template_caps);
      return FALSE;
    }

    allowed_caps = gst_caps_make_writable (allowed_caps);
    allowed_caps = gst_caps_fixate (allowed_caps);
    s = gst_caps_get_structure (allowed_caps, 0);
    allowed_mime_name = gst_structure_get_name (s);

    if (!g_strcmp0 (allowed_mime_name, "video/x-h265"))
    {
      enc->codec.id = CODEC_ID_H265;
    } else {
      enc->codec.id = CODEC_ID_H264;
    }

    gst_caps_unref (allowed_caps);
  }

  gst_caps_unref (template_caps);

  if (!gst_customvenc_init_enc (encoder))
    return FALSE;

  return TRUE;
}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn
gst_customvenc_handle_frame (GstVideoEncoder * video_enc,
    GstVideoCodecFrame * frame)
{
  GstCustomEnc *enc = GST_CUSTOMVENC (video_enc);
  GstFlowReturn ret;

  if (G_UNLIKELY (enc->codec.handle == 0))
    goto not_inited;

  ret = gst_customvenc_encode_frame (enc, frame);

  /* input buffer is released later on */
  return ret;

/* ERRORS */
not_inited:
  {
    GST_WARNING_OBJECT (enc, "Got buffer before set_caps was called");
    return GST_FLOW_NOT_NEGOTIATED;
  }
}

static GstFlowReturn
gst_customvenc_encode_frame (GstCustomEnc * enc,
    GstVideoCodecFrame * frame)
{

  /* 下面部份使用video encoder编码帧(调用相应的lib库), 不同的
   * venc又所不同, 这里还支持使用dma buffer, 这段代码就不放出了
   */

#if 0
  vl_frame_type_t frame_type = FRAME_TYPE_AUTO;
  GstVideoInfo *info = &enc->input_state->info;
  GstMapInfo map;
  gint encode_data_len = -1;

  if (G_UNLIKELY (enc->codec.handle == 0)) {
    if (frame)
      gst_video_codec_frame_unref (frame);
    return GST_FLOW_NOT_NEGOTIATED;
  }

  if (frame) {
    if (GST_VIDEO_CODEC_FRAME_IS_FORCE_KEYFRAME (frame)) {
      GST_INFO_OBJECT (enc, "Forcing key frame");
      frame_type = FRAME_TYPE_IDR;
    }
  }

  vl_buffer_info_t inbuf_info;
  vl_buffer_info_t retbuf_info;

  GstMemory *memory = gst_buffer_get_memory(frame->input_buffer, 0);
  gboolean is_dmabuf = gst_is_dmabuf_memory(memory);
  gint fd = -1;
  GstMapInfo minfo;

  if (is_dmabuf) {
    fd = gst_dmabuf_memory_get_fd(memory);
    gst_memory_unref(memory);
  } else {
    gst_memory_unref(memory);

    if (enc->imgproc.input.memory == NULL) {
      enc->imgproc.input.memory =
        gst_allocator_alloc(enc->dmabuf_alloc, info->size, NULL);
      if (enc->imgproc.input.memory == NULL) {
        GST_ERROR_OBJECT(enc, "failed to allocate new dma buffer");
        return GST_FLOW_ERROR;
      }
      enc->imgproc.input.fd = gst_dmabuf_memory_get_fd(encoder->imgproc.input.memory);
    }

    memory = enc->imgproc.input.memory;
    fd = enc->imgproc.input.fd;
    if (gst_memory_map(memory, &minfo, GST_MAP_WRITE)) {
      GstVideoFrame video_frame;
      gst_video_frame_map(&video_frame, info, frame->input_buffer, GST_MAP_READ);
      guint8 *pixel = GST_VIDEO_FRAME_PLANE_DATA (&video_frame, 0);
      memcpy(minfo.data, pixel, info->size);
      gst_video_frame_unmap (&video_frame);
      gst_memory_unmap(memory, &minfo);
    }
  }


  if (enc->imgproc.handle) {
    // format conversation needed
    if (enc->dmabuf_alloc == NULL) {
      enc->dmabuf_alloc = gst_amlion_allocator_obtain();
    }

    struct imgproc_buf inbuf, outbuf;
    inbuf.fd = fd;
    inbuf.is_ionbuf = gst_is_amlionbuf_memory(memory);

    if (enc->imgproc.output.memory == NULL) {
      enc->imgproc.output.memory = gst_allocator_alloc(
          enc->dmabuf_alloc, encoder->imgproc.outbuf_size, NULL);
      if (enc->imgproc.output.memory == NULL) {
        GST_ERROR_OBJECT(enc, "failed to allocate new dma buffer");
        return GST_FLOW_ERROR;
      }
      enc->imgproc.output.fd = gst_dmabuf_memory_get_fd(encoder->imgproc.output.memory);
    }

    fd = enc->imgproc.output.fd;

    outbuf.fd = fd;
    outbuf.is_ionbuf = TRUE;

    struct imgproc_pos inpos = {
        0, 0, info->width, info->height, info->width, info->height};
    struct imgproc_pos outpos = {
        0, 0, info->width, info->height, info->width, info->height};

    imgproc_crop(enc->imgproc.handle, inbuf, inpos,
                      GST_VIDEO_INFO_FORMAT(info), outbuf, outpos,
                      GST_VIDEO_FORMAT_NV12);
  }

  memset(&inbuf_info, 0, sizeof(vl_buffer_info_t));
  inbuf_info.buf_type = DMA_TYPE;
  inbuf_info.buf_info.dma_info.shared_fd[0] = fd;
  inbuf_info.buf_info.dma_info.shared_fd[1] = -1;
  inbuf_info.buf_info.dma_info.shared_fd[2] = -1;
  inbuf_info.buf_info.dma_info.num_planes = 1;

  encoding_metadata_t meta =
      vl_multi_enc_encode(encoder->codec.handle, frame_type,
                              enc->codec.buf, &inbuf_info, &retbuf_info);

  if (!meta.is_valid) {
    if (frame) {
      GST_ELEMENT_ERROR (enc, STREAM, ENCODE, ("Encode v frame failed."),
          ("gst_customvenc_encode return code=%d", encode_data_len));
      gst_video_codec_frame_unref (frame);
      return GST_FLOW_ERROR;
    } else {
      return GST_FLOW_EOS;
    }
  }

  if (frame) {
    gst_video_codec_frame_unref (frame);
  }

  //frame = gst_video_enc_get_frame (GST_VIDEO_ENCODER (encoder), input_frame->system_frame_number);
  frame = gst_video_enc_get_oldest_frame (GST_VIDEO_ENCODER (encoder));
  if (!frame) {
    gst_video_codec_frame_unref (frame);
    return GST_FLOW_ERROR;
  }

  frame->output_buffer = gst_video_enc_allocate_output_buffer(
      GST_VIDEO_ENCODER(enc), meta.encoded_data_length_in_bytes);
  gst_buffer_map(frame->output_buffer, &map, GST_MAP_WRITE);
  memcpy (map.data, enc->codec.buf, meta.encoded_data_length_in_bytes);
  gst_buffer_unmap (frame->output_buffer, &map);

  frame->dts = frame->pts;

  GST_LOG_OBJECT (enc,
      "output: dts %" G_GINT64_FORMAT " pts %" G_GINT64_FORMAT,
      (gint64) frame->dts, (gint64) frame->pts);

  if (frame_type == FRAME_TYPE_IDR) {
    GST_DEBUG_OBJECT (enc, "Output keyframe");
    GST_VIDEO_CODEC_FRAME_SET_SYNC_POINT (frame);
  } else {
    GST_VIDEO_CODEC_FRAME_UNSET_SYNC_POINT (frame);
  }
#endif

  return gst_video_enc_finish_frame ( GST_VIDEO_ENCODER(encoder), frame);

}

static void
gst_customvenc_get_property (GObject * obj, guint id,
    GValue * val, GParamSpec * spec)
{
  GstCustomEnc *enc = GST_CUSTOMVENC (obj);

  GST_OBJECT_LOCK (enc);
  switch (id) {
    case P_GOP:
      g_value_set_int (val, enc->gop);
      break;
    case P_FRAMERATE:
      g_value_set_int (val, enc->framerate);
      break;
    case P_BITRATE:
      g_value_set_int (val, enc->bitrate);
      break;
    case P_MIN_BUFFERS:
      g_value_set_int (val, enc->min_buffers);
      break;
    case P_MAX_BUFFERS:
      g_value_set_int (val, enc->max_buffers);
      break;
    case P_ENCODER_BUFSIZE:
      g_value_set_int (val, enc->encoder_bufsize / 1024);
      break;
    default:
      G_OBJECT_WARN_INVALID_PERTY_ID (obj, id, spec);
      break;
  }
  GST_OBJECT_UNLOCK (enc);
}

static void
gst_customvenc_set_property (GObject * obj, guint id,
        const GValue * val, GParamSpec * spec)
{
    GstCustomEnc *enc = GST_CUSTOMVENC (obj);

    GST_OBJECT_LOCK (enc);

    switch (id) {
        case P_GOP: {
            gint gop = g_value_get_int (val);
            if (gop != enc->gop) {
                enc->gop = gop;
            if (enc->codec.handle) {
                gint ret = vl_video_enc_change_gop (enc->codec.handle, 30, enc->gop);
                GST_DEBUG ("Change gop:%d", enc->gop);
                if (ret != 0)
                    GST_DEBUG ("change gop error, ret:%d", ret);
                }
            }
        } break;
        case P_FRAMERATE:
            enc->framerate = g_value_get_int (val);
            break;
        case P_BITRATE: {
            gint bitrate = g_value_get_int (val);
            if (bitrate != enc->bitrate) {
                enc->bitrate = bitrate;
                if (enc->codec.handle) {
                    gint ret = vl_video_enc_change_bitrate (enc->codec.handle, enc->bitrate * 1000);  // 内部encoder接口
                    GST_DEBUG ("Change bitrate:%d", enc->bitrate);
                    if (ret != 0)
                        GST_DEBUG ("change bitrate error, ret:%d", ret);
                }
            }
        } break;
        case P_MIN_BUFFERS:
          enc->min_buffers = g_value_get_int (val);
          break;
        case P_MAX_BUFFERS:
          enc->max_buffers = g_value_get_int (val);
          break;
        case P_ENCODER_BUFSIZE:
          enc->enc_bufsize = g_value_get_int (val) * 1024;
          break;
        default:
            G_OBJECT_WARN_INVALID_PERTY_ID (obj, id, spec);
            break;
    }


    GST_OBJECT_UNLOCK (enc);
    return;
}

static gboolean
plugin_init (GstPlugin * customvenc){

    // 用于用户debug
    GST_DEBUG_CATEGORY_INIT (gst_customvenc_debug, "customvenc", 0,
        "customized video encoding element");

    return gst_element_register (customvenc, "customvenc",
        GST_RANK_PRIMARY, GST_TYPE_CUSTOMVENC);
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR, GST_VERSION_MINOR, customvenc,
    "customized video encoder plugins", plugin_init,
    VERSION, "LGPL", "customized video ecoding",
    "Local"
)
