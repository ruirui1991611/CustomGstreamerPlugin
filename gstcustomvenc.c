/**
 * SECTION:element-customvenc
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v -m fakesrc ! customvenc ! fakesink silent=TRUE
 * ]|
 * </refsect2>
 */

#include <gmodule.h>
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

#define P_ROI_ID_DEFAULT 0
#define P_ROI_ENABLED_DEFAULT TRUE
#define P_ROI_WIDTH_DEFAULT 0.00
#define P_ROI_HEIGHT_DEFAULT 0.00
#define P_ROI_X_DEFAULT 0.00
#define P_ROI_Y_DEFAULT 0.00
#define P_ROI_QUALITY_DEFAULT 51

#define P_IDR_PERIOD_DEFAULT 30
#define P_FRAMERATE_DEFAULT 30
#define P_BITRATE_DEFAULT 2000
#define P_BITRATE_MAX 12000
#define P_MIN_BUFFERS_DEFAULT 4
#define P_MAX_BUFFERS_DEFAULT 6
#define P_ENCODER_BUFFER_SIZE_DEFAULT 2048
#define P_ENCODER_BUFFER_SIZE_MIN 1024
#define P_ENCODER_BUFFER_SIZE_MAX 4096


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

struct roi_related {
    struct listnode list;
    gint id;
    gint quality;
    struct {
        gfloat left;
        gfloat top;
        gfloat width;
        gfloat height;
    } location;
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

/*
 * venc_image_fmt类型是 video encoder lib库里的
 * 此函数是用于将gstreamer定义的的image format转
 * 化成venc支持的类型
 */

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


/* 当上游stream来到时, gstreamer框架会调用此函数去
 * 询问上游pad的capability, 以此为依据来进行连接
 */
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

/*
 * 初始化必要的callback函数, gstreamer框架负责调用
 * Install 必要的property
 */
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

    enc_class->start = GST_DEBUG_FUNCPTR (gst_customvenc_start);
    enc_class->stop = GST_DEBUG_FUNCPTR (gst_customvenc_stop);
    enc_class->flush = GST_DEBUG_FUNCPTR (gst_customvenc_flush);
    enc_class->set_format = GST_DEBUG_FUNCPTR (gst_customvenc_set_format);
    enc_class->sink_query = GST_DEBUG_FUNCPTR (gst_customvenc_sink_query);
    enc_class->handle_frame =
        GST_DEBUG_FUNCPTR (gst_customvenc_handle_frame);

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

    // roi相关property
    g_obj_class_install_property (obj_class, P_ROI_ENABLED,
        g_param_spec_boolean ("roi-enabled", "roi-enabled", "Enable/Disable roi",
            P_ROI_ENABLED_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property(obj_class, P_ROI_ID,
        g_param_spec_int("roi-id", "roi-id", "Current roi index",
            0, G_MAXINT32, P_ROI_ID_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_ROI_WIDTH,
        g_param_spec_float("roi-width", "roi-width", "roi rectangle",
            0.00, G_MAXFLOAT, P_ROI_WIDTH_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_ROI_HEIGHT,
        g_param_spec_float("roi-height", "roi-height", "roi rectangle",
            0.00, G_MAXFLOAT, P_ROI_HEIGHT_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_ROI_X,
        g_param_spec_float("roi-x", "roi-x", "horizontal start position of the roi rectangle",
            0.00, G_MAXFLOAT, P_ROI_X_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_ROI_Y,
        g_param_spec_float("roi-y", "roi-y", "vertical start position of the roi rectangle",
            0.00, G_MAXFLOAT, P_ROI_Y_DEFAULT,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

    g_obj_class_install_property (obj_class, P_ROI_QUALITY,
        g_param_spec_int("roi-quality", "roi-quality", "roi quality",
            0, 51, P_ROI_QUALITY_DEFAULT,
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

/*
 * 初始化venc需要的变量
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

/* 根据id 遍历链表，获取对应的指针地址 */
static struct roi_param *get_roi_param(GstCustomEnc *enc, gint id) {
  GstCustomEnc *self = GST_CUSTOMVENC (enc);
  struct roi_param *ret = NULL;
  if (!list_empty (&self->roi.param)) {
    struct listnode *pos;
    list_for_each (pos, &self->roi.param) {
      struct roi_param *param =
        list_entry (pos, struct roi_param, list);
      if (param->id == id) {
        ret = param;
      }
    }
  }
  if (ret == NULL) {
    ret = g_new(struct roi_param, 1);
    list_init (&ret->list);
    list_add_tail(&self->roi.param, &ret->list);
    ret->id = id;
    ret->location.left = P_ROI_X_DEFAULT;
    ret->location.top = P_ROI_Y_DEFAULT;
    ret->location.width = P_ROI_WIDTH_DEFAULT;
    ret->location.height = P_ROI_HEIGHT_DEFAULT;
    ret->quality = P_ROI_QUALITY_DEFAULT;
  }
  return ret;
}

/*
 * 计算用于设置roi的buffer, venc接口需要使用该数据
 */
static void
gst_customvenc_set_roi_buf(guchar* buffer, gint buf_w, gint buf_h,
    struct roi_related *param, gint in_fr_w, gint in_fr_h, gint block_size) {
    if (buffer == NULL || param == NULL) return;

    gint left = param->location.left * in_fr_w;
    gint top = param->location.top * in_fr_h;
    gint width = param->location.width * in_fr_w;
    gint height = param->location.height * in_fr_h;

    gint right = left + width;
    gint bottom = top + height;

    gint limit = block_size / 2;

    gint start_row = top / block_size;
    gint start_col = left / block_size;
    if ((left % block_size) > limit) start_col += 1;
    if ((top % block_size) > limit) start_row += 1;

    gint stop_row = bottom / block_size;
    gint stop_col = right / block_size;
    if ((right % block_size) >= limit) stop_col += 1;
    if ((bottom % block_size) >= limit) stop_row += 1;

    if (start_row <= stop_row && start_col <= stop_col) {
        for (int i_row = start_row; i_row < stop_row; i_row++) {
            for (int j_col = start_col; j_col < stop_col; j_col++) {
                buffer[i_row * buf_w + j_col] = (param->quality);
            }
        }
    }
}

/*
 * gst_customvenc_set_roi: 设置roi
 * @enc:  set roi.
 * Set roi value
 */
static gboolean
gst_customvenc_set_roi(GstCustomEnc * encoder)
{
    GstVideoInfo *info;

    if (!encoder->input_state) {
        GST_DEBUG_OBJECT (encoder, "Have no input state yet");
        return FALSE;
    }

    info = &encoder->input_state->info;
    // 进来frame的宽度和高度
    gint in_fr_w = info->width;
    gint in_fr_h = info->height;

    gint buf_w = encoder->roi.buffer_info.width;
    gint buf_h = encoder->roi.buffer_info.height;

    if (encoder->roi.enabled) {
        struct listnode *pos = NULL;
        struct roi_related *param = NULL;
        list_for_each(pos, &encoder->roi.param) {
            param = list_entry(pos, struct roi_related, list);
            gst_customvenc_set_roi_buf(
                encoder->roi.buf_info.data,
                buf_w, buf_h,
                param,
                in_fr_w,
                in_fr_h,
                encoder->roi.block_size
            );
        }
    }

    gint ret;
    if (encoder->codec.handle) {
        /* 调用encoder lib库接口更新qp值 <==> 更新roi value */
        if ((ret = venc_update_qp_val(
            encoder->codec.handle,
            encoder->roi.buffer_info.data,
            buf_w * buf_h)) != 0) {
            return FALSE;
        }
    }
}

/*
 * once_set_roi() 函数
 * 当用户通过set_property修改某些roi的参数时(enable, quality)
 * 需要重新调用gst_customvenc_set_roi()来设置一下, 使用g_timeout_add()
 * 来保证再超时范围内只调用一次即可
 */
static gboolean
once_set_roi(GstCustomEnc * self) {
  if (self != NULL) {
      gst_customvenc_set_roi (self);
  }

  return G_SOURCE_REMOVE;
}


/*
 * gst_customvenc_init_enc()
 * @encoder: 初始化video encoder
 */
static gboolean
gst_customvenc_init_enc (GstCustomEnc * encoder)
{
    GstVideoInfo *info;

    if (!encoder->input_state) {
        GST_DEBUG_OBJECT (enc, "Have no input state yet");
        return FALSE;
    }

    info = &encoder->input_state->info;

    /* 确保 video encoder 被关闭 */
    gst_customvenc_close_enc (encoder);

    GST_OBJECT_LOCK (encoder);


    GST_OBJECT_UNLOCK (encoder);

    /* 初始化video encoder */
    venc_info_t venc_info;
    memset (&venc_info, 0, sizeof(venc_info_t));

    venc_info.width = info->width;
    venc_info.height = info->height;
    venc_info.frame_rate = enc->framerate;
    venc_info.bit_rate = enc->bitrate * 1000;
    venc_info.gop = enc->gop;
    venc_info.img_format = convert_image_format(GST_VIDEO_INFO_FORMAT(info));
    venc_info.prepend_spspps_to_idr_frames = TRUE;
    venc_info.feature |= 0x1;  // ROI功能使能
    venc_info.feature |= 0x2;  // 修改参数功能使能(修改gop和bitrate)

    params_of_qp_t qp_info;
    memset(&qp_info, 0, sizeof(params_of_qp_t));

    qp_info.min_val = 0;
    qp_info.max_val = 51;
    qp_info.I_base = 30;
    qp_info.I_min = 0;
    qp_info.I_max = 51;
    qp_info.P_base = 30;
    qp_info.P_min = 0;
    qp_info.P_max = 51;

    enc->codec.handle = venc_init(encoder->codec.id, venc_info, &qp_info);

    if (encoder->codec.handle == 0) {
        GST_ELEMENT_ERROR (encoder, STREAM, ENCODE,
            ("Initialize venc error"), (NULL));
        return FALSE;
    }

    if (!gst_customvenc_set_roi (encoder)) {
        return FALSE;
    }

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
        venc_destroy(encoder->codec.handle);
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
            enc->codec.id = H265;
        } else {
            enc->codec.id = H264;
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

    if (G_UNLIKELY (enc->codec.handle == 0)) {
        GST_WARNING_OBJECT (enc, "Internal Problem");
        return GST_FLOW_NOT_NEGOTIATED;
    }

    ret = gst_customvenc_encode_frame (enc, frame);

    return ret;

}

static GstFlowReturn
gst_customvenc_encode_frame (GstCustomEnc * enc,
    GstVideoCodecFrame * frame) {

    /* 下面部份使用video encoder编码帧(调用相应的lib库), 不同的
     * venc有所不同, 这里还支持使用dma buffer, 具体代码就不放出了
     */

    return gst_video_enc_finish_frame ( GST_VIDEO_ENCODER(encoder), frame);
}

static void
gst_customvenc_get_property (GObject * obj, guint id,
    GValue * val, GParamSpec * spec) {
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
        case P_ROI_ENABLED:
            g_value_set_boolean (val, enc->roi.enabled);
            break;
        case P_ROI_ID:
            g_value_set_int (val, enc->roi.id);
            break;
        case P_ROI_WIDTH: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                g_value_set_float (val, param->location.width);
            } break;
        case P_ROI_HEIGHT: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
            g_value_set_float (val, param->location.height);
            } break;
        case P_ROI_X: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                g_value_set_float (val, param->location.left);
            } break;
        case P_ROI_Y: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                g_value_set_float (val, param->location.top);
            } break;
        case P_ROI_QUALITY: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                g_value_set_int (val, param->quality);
            } break;
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
    gboolean is_set_roi = false;

    GST_OBJECT_LOCK (enc);

    switch (id) {
        case P_GOP: {
            gint gop = g_value_get_int (val);
            if (gop != enc->gop) {
                enc->gop = gop;
            if (enc->codec.handle) {
                gint ret = venc_change_gop (enc->codec.handle, 30, enc->gop);
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
                    gint ret = venc_change_bitrate (enc->codec.handle, enc->bitrate * 1000);  // 内部encoder lib接口
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
        case P_ROI_ENABLED: {
                gboolean enabled = g_value_get_boolean (val);
                if (!enabled) {
                    roi_list_clear(enc);
                    memset(enc->roi.buffer_info.data,
                    P_ROI_QUALITY_DEFAULT,
                    enc->roi.buffer_info.width * enc->roi.buffer_info.height);
                }
                if (enabled != enc->roi.enabled) {
                    enc->roi.enabled = enabled;
                    is_set_roi = true;
                }
            } break;
        case P_ROI_ID: {
                gint ID = g_value_get_int (val);
                if (ID != enc->roi.ID) {
                    enc->roi.id = ID;
                }
            } break;
        case P_ROI_WIDTH: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                param->location.width = g_value_get_float (val);
            } break;
        case P_ROI_HEIGHT: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                param->location.height = g_value_get_float (val);
            } break;
        case P_ROI_X: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                param->location.left = g_value_get_float (val);
            } break;
        case P_ROI_Y: {
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                param->location.top = g_value_get_float (val);
            } break;
        case P_ROI_QUALITY: {
                gint quality = g_value_get_int (val);
                struct roi_param *param = get_roi_param(enc, enc->roi.id);
                if (quality != param->quality) {
                    param->quality = quality;
                    is_set_roi = true;
                }
            } break;
        default:
            G_OBJECT_WARN_INVALID_PERTY_ID (obj, id, spec);
            break;
    }

    if (is_set_roi) {
        g_timeout_add(500, (GSourceFunc)once_set_roi, (gpointer)enc);
    }

    GST_OBJECT_UNLOCK (enc);
    return;
}

static gboolean
plugin_init (GstPlugin * customvenc){

    // 方便用户层进行debug
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
